#include "motor_crc.h"
#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/bms_cmd.hpp"
#include "unitree_go/msg/low_cmd.hpp"
#include "unitree_go/msg/motor_cmd.hpp"

#include "unitree/robot/channel/channel_factory.hpp"
#include "unitree/robot/go2/robot_state/robot_state_api.hpp"
#include "unitree/robot/go2/robot_state/robot_state_client.hpp"
#include "unitree/robot/go2/robot_state/robot_state_error.hpp"

#include "unitree/robot/go2/sport/sport_client.hpp"
#include "unitree_go/msg/low_state.hpp"

#include <chrono>
#include <cstdint>
#include <functional>
#include <thread>
#include <vector>

// Create a low_level_cmd_sender class for low state receive
class low_level_cmd_sender : public rclcpp::Node {
public:
  low_level_cmd_sender() : Node("low_level_cmd_sender") {

    sport_client.SetTimeout(5.0f);
    sport_client.Init();

    init_robot_state();

    state_suber = this->create_subscription<unitree_go::msg::LowState>(
        "/lowstate", 10,
        std::bind(&low_level_cmd_sender::lowstate_callback, this,
                  std::placeholders::_1));

    turn_off_mode();

    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // the cmd_puber is set to subscribe "/lowcmd" topic
    cmd_puber = this->create_publisher<unitree_go::msg::LowCmd>("/lowcmd", 10);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(int(dt * 1000)),
        std::bind(&low_level_cmd_sender::timer_callback, this));

    init_cmd();
  }

private:
  double dt = 0.002;
  double runing_time = 0.0;
  double phase = 0.0;

  bool first_run = true;
  bool end = false;

  // Variable para guardar el servicio que debemos apagar/encender modificar
  // para si hay mas de 1 servicio encendido apagar todos y guardar todos
  std::vector<std::string> active_motion_services;

  double stand_up_joint_pos[12] = {
      0.00571868, 0.608813, -1.21763, -0.00571868, 0.608813, -1.21763,
      0.00571868, 0.608813, -1.21763, -0.00571868, 0.608813, -1.21763};
  double initial_joint_pos[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

  rclcpp::TimerBase::SharedPtr timer_; // ROS2 timer
  rclcpp::Publisher<unitree_go::msg::LowCmd>::SharedPtr
      cmd_puber; // ROS2 Publisher
  rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr state_suber;

  unitree::robot::go2::RobotStateClient client;
  unitree::robot::go2::SportClient sport_client;

  unitree_go::msg::LowCmd low_cmd;
  unitree_go::msg::LowState low_state;

  void init_robot_state() {
    client.SetTimeout(5.0f);
    client.Init();

    std::vector<unitree::robot::go2::ServiceState> serviceList;
    int32_t ret = client.ServiceList(serviceList);

    if (ret != 0) {
      RCLCPP_ERROR(this->get_logger(), "Error al obtener servicios: %d", ret);
      return;
    }

    std::cout << "\n--- LISTA DE SERVICIOS DETECTADOS ---" << std::endl;
    printf("%-30s %-10s %-10s\n", "Nombre", "Status", "Protect");

    for (const auto &svc : serviceList) {
      printf("%-30s %-10d %-10d\n", svc.name.c_str(), svc.status, svc.protect);

      bool is_motion_service = (svc.name == "mcf" || svc.name == "sport_mode");
      if (is_motion_service && svc.status == 1 && svc.protect == 0) {
        active_motion_services.push_back(svc.name);
        RCLCPP_INFO(this->get_logger(),
                    "Servicio de movimiento activo guardado: %s",
                    svc.name.c_str());
      }
    }
    std::cout << "-------------------------------------\n" << std::endl;

    if (active_motion_services.empty()) {
      RCLCPP_WARN(this->get_logger(),
                  "No se detectó ningún servicio de movimiento activo.");
    }
  }

  void turn_off_mode() {
    if (active_motion_services.empty()) {
      RCLCPP_INFO(this->get_logger(), "Sin servicios que apagar, saltando.");
      return;
    }
    for (const auto &svc_name : active_motion_services) {
      int32_t status;
      client.ServiceSwitch(svc_name, 0, status);
      RCLCPP_INFO(this->get_logger(), "Servicio parado: %s | status salida: %d",
                  svc_name.c_str(), status);
    }
  }

  void turn_on_mode() {
    for (const auto &svc_name : active_motion_services) {
      int32_t status;
      client.ServiceSwitch(svc_name, 1, status);
      RCLCPP_INFO(this->get_logger(),
                  "Servicio reactivado: %s | status salida: %d",
                  svc_name.c_str(), status);
    }
  }

  void lowstate_callback(const unitree_go::msg::LowState::SharedPtr msg) {
    low_state = *msg;

    if (first_run) {
      for (int i = 0; i < 12; i++) {
        initial_joint_pos[i] = msg->motor_state[i].q;
      }
      first_run = false;
      RCLCPP_INFO(
          this->get_logger(),
          "Posición inicial capturada con éxito. Iniciando movimiento...");
    }
  }

  void finish_sequence() {
    // secuencia completa de restauración en hilo separado para no
    // bloquear el nodo ROS2 mientras esperamos que el robot ejecute los
    // movimientos
    std::thread([this]() {
      // 1. Reactivar el servicio de alto nivel antes de usar SportClient
      turn_on_mode();
      std::this_thread::sleep_for(std::chrono::seconds(1));

      // 2. StandDown con alto nivel
      // sport_client.StandDown();
      RCLCPP_INFO(this->get_logger(), "StandDown ejecutado.");
      std::this_thread::sleep_for(std::chrono::seconds(3));

      // 3. Damp: motores en modo pasivo, el robot queda "suelto"
      sport_client.Damp();
      RCLCPP_INFO(this->get_logger(), "Damp ejecutado. Cerrando...");
      std::this_thread::sleep_for(std::chrono::seconds(1));

      rclcpp::shutdown();
    }).detach(); // detach para que no bloquee
  }

  void timer_callback() {

    if (first_run)
      return;

    if (end)
      return;

    runing_time += dt;

    if (runing_time < 3.0) {
      // Stand up in first 3 second

      // Total time for standing up or standing down is about 1.2s
      phase = tanh(runing_time / 1.2);
      for (int i = 0; i < 12; i++) {
        printf("Joint: %d ===> %lf", i,
               phase * stand_up_joint_pos[i] +
                   (1 - phase) * initial_joint_pos[i]);
        // low_cmd.motor_cmd[i].q =
        //    phase * stand_up_joint_pos[i] + (1 - phase) *
        //    initial_joint_pos[i];
        // low_cmd.motor_cmd[i].dq = 0;
        // low_cmd.motor_cmd[i].kp = phase * 50.0 + (1 - phase) * 20.0;
        //  low_cmd.motor_cmd[i].kd = 3.5;
        //  low_cmd.motor_cmd[i].tau = 0;
      }
    } else {
      // Then stand down
      phase = tanh((runing_time - 3.0) / 1.2);
      for (int i = 0; i < 12; i++) {
        printf("Joint: %d ===> %lf", i,
               phase * initial_joint_pos[i] +
                   (1 - phase) * stand_up_joint_pos[i]);
        // low_cmd.motor_cmd[i].q =
        //    phase * initial_joint_pos[i] + (1 - phase) *
        //    stand_up_joint_pos[i];
        // low_cmd.motor_cmd[i].dq = 0;
        // low_cmd.motor_cmd[i].kp = 50;
        // low_cmd.motor_cmd[i].kd = 3.5;
        // low_cmd.motor_cmd[i].tau = 0;
      }
    }

    get_crc(low_cmd);            // Check motor cmd crc
    cmd_puber->publish(low_cmd); // Publish lowcmd message

    if (runing_time > 5.0) {
      end = true;
      finish_sequence();
      return;
    }
  }

  void init_cmd() {
    for (int i = 0; i < 20; i++) {
      low_cmd.motor_cmd[i].mode = 0x01; // Set toque mode, 0x00 is passive mode
      low_cmd.motor_cmd[i].q = PosStopF;
      low_cmd.motor_cmd[i].kp = 0;
      low_cmd.motor_cmd[i].dq = VelStopF;
      low_cmd.motor_cmd[i].kd = 0;
      low_cmd.motor_cmd[i].tau = 0;
    }
  }
};

int main(int argc, char **argv) {
  std::cout << "Press enter to start";
  std::cin.get();

  if (argc > 1) {
    // unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
  } else {
    // unitree::robot::ChannelFactory::Instance()->Init(0);  // usa entorno
  }

  rclcpp::init(argc, argv);
  auto node = std::make_shared<low_level_cmd_sender>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
