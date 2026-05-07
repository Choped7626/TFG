#include "motor_crc.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "unitree_go/msg/low_cmd.hpp"
#include "unitree_go/msg/low_state.hpp"

class low_level_cmd_sender : public rclcpp::Node {
public:
  low_level_cmd_sender() : Node("low_level_cmd_sender") {

    state_suber = this->create_subscription<unitree_go::msg::LowState>(
        "/lowstate", 10,
        std::bind(&low_level_cmd_sender::lowstate_callback, this,
                  std::placeholders::_1));

    // Suscribirse al estado de servicios publicado por state_manager
    services_suber = this->create_subscription<std_msgs::msg::String>(
        "/unitree/active_services", 10,
        [this](std_msgs::msg::String::SharedPtr msg) {
          active_services_str = msg->data;
        });

    // Clientes para llamar a state_manager
    sport_mode_client =
        this->create_client<std_srvs::srv::SetBool>("/unitree/sport_mode");
    damp_client = this->create_client<std_srvs::srv::SetBool>("/unitree/damp");

    cmd_puber = this->create_publisher<unitree_go::msg::LowCmd>("/lowcmd", 10);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(int(dt * 1000)),
        std::bind(&low_level_cmd_sender::timer_callback, this));

    // Apagar sport_mode al inicio usando un hilo separado
    std::thread([this]() {
      RCLCPP_INFO(this->get_logger(),
                  "Esperando a desactivar sport_mode para arrancar...");

      // Se quedará aquí bloqueado infinitamente (gracias al while que pusimos
      // antes) hasta que el nodo state_manager responda con un success.
      if (call_sport_mode(false)) {
        safe_to_move = true; // ¡Damos luz verde al movimiento!
        RCLCPP_INFO(
            this->get_logger(),
            "Estados desactivados correctamente. Comenzando movimiento.");
      }
    }).detach();

    init_cmd();
  }

private:
  bool call_sport_mode(bool enable) {
    // Bucle infinito que espera hasta que el servicio aparezca
    while (!sport_mode_client->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(),
                     "Nodo interrumpido mientras esperaba al state_manager.");
        return false;
      }
      RCLCPP_INFO(
          this->get_logger(),
          "state_manager no disponible, esperando a que se encienda...");
    }

    auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
    req->data = enable;

    auto future = sport_mode_client->async_send_request(req);
    try {
      auto res = future.get();
      RCLCPP_INFO(this->get_logger(), "sport_mode(%d): %s", enable,
                  res->message.c_str());
      return res->success;
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Excepción al llamar sport_mode: %s",
                   e.what());
      return false;
    }
  }

  bool call_damp() {
    // Bucle infinito que espera hasta que el servicio aparezca
    while (!damp_client->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(),
                     "Nodo interrumpido mientras esperaba al state_manager.");
        return false;
      }
      RCLCPP_INFO(this->get_logger(),
                  "state_manager no disponible para damp, esperando...");
    }

    auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
    req->data = true;

    auto future = damp_client->async_send_request(req);
    try {
      auto res = future.get();
      RCLCPP_INFO(this->get_logger(), "Damp: %s", res->message.c_str());
      return res->success;
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Excepción al llamar damp: %s",
                   e.what());
      return false;
    }
  }

  void finish_sequence() {
    std::thread([this]() {
      // Espera a que sport_mode responda antes de continuar
      if (call_sport_mode(true)) {
        // Solo llama a damp si sport_mode se activó con éxito
        call_damp();
      } else {
        RCLCPP_ERROR(this->get_logger(),
                     "Fallo en sport_mode, cancelando damp.");
      }
      rclcpp::shutdown();
    }).detach();
  }

  void lowstate_callback(const unitree_go::msg::LowState::SharedPtr msg) {
    low_state = *msg;
    if (first_run) {
      for (int i = 0; i < 12; i++)
        initial_joint_pos[i] = msg->motor_state[i].q;
      first_run = false;
      RCLCPP_INFO(this->get_logger(), "Posición inicial capturada.");
    }
  }

  void timer_callback() {

    if (!safe_to_move)
      return;
    if (first_run || end)
      return;

    runing_time += dt;

    if (runing_time < 3.0) {
      phase = tanh(runing_time / 1.2);
      for (int i = 0; i < 12; i++) {
        low_cmd.motor_cmd[i].q =
            phase * stand_up_joint_pos[i] + (1 - phase) * initial_joint_pos[i];
        low_cmd.motor_cmd[i].dq = 0;
        low_cmd.motor_cmd[i].kp = phase * 50.0 + (1 - phase) * 20.0;
        low_cmd.motor_cmd[i].kd = 3.5;
        low_cmd.motor_cmd[i].tau = 0;
      }
    } else {
      phase = tanh((runing_time - 3.0) / 1.2);
      for (int i = 0; i < 12; i++) {
        low_cmd.motor_cmd[i].q =
            phase * initial_joint_pos[i] + (1 - phase) * stand_up_joint_pos[i];
        low_cmd.motor_cmd[i].dq = 0;
        low_cmd.motor_cmd[i].kp = 50;
        low_cmd.motor_cmd[i].kd = 3.5;
        low_cmd.motor_cmd[i].tau = 0;
      }
    }

    get_crc(low_cmd);
    cmd_puber->publish(low_cmd);

    if (runing_time > 5.0) {
      end = true;
      finish_sequence();
    }
  }

  void init_cmd() {
    for (int i = 0; i < 20; i++) {
      low_cmd.motor_cmd[i].mode = 0x01;
      low_cmd.motor_cmd[i].q = PosStopF;
      low_cmd.motor_cmd[i].kp = 0;
      low_cmd.motor_cmd[i].dq = VelStopF;
      low_cmd.motor_cmd[i].kd = 0;
      low_cmd.motor_cmd[i].tau = 0;
    }
  }

  double dt = 0.002, runing_time = 0.0, phase = 0.0;
  bool safe_to_move = false; // <--- NUEVA VARIABLE DE SEGURIDAD
  bool first_run = true, end = false;
  std::string active_services_str = "";

  double stand_up_joint_pos[12] = {
      0.00571868, 0.608813, -1.21763, -0.00571868, 0.608813, -1.21763,
      0.00571868, 0.608813, -1.21763, -0.00571868, 0.608813, -1.21763};
  double initial_joint_pos[12] = {0};

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<unitree_go::msg::LowCmd>::SharedPtr cmd_puber;
  rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr state_suber;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr services_suber;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr sport_mode_client;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr damp_client;

  unitree_go::msg::LowCmd low_cmd;
  unitree_go::msg::LowState low_state;
};

int main(int argc, char **argv) {
  std::cout << "Press enter to start";
  std::cin.get();
  rclcpp::init(argc, argv); // sin ChannelFactory aquí
  rclcpp::spin(std::make_shared<low_level_cmd_sender>());
  rclcpp::shutdown();
  return 0;
}
