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

    RCLCPP_INFO(this->get_logger(), "Creado subscriptor /lowstate");

    // Suscribirse al estado de servicios publicado por state_manager
    services_suber = this->create_subscription<std_msgs::msg::String>(
        "/unitree/active_services", 10,
        [this](std_msgs::msg::String::SharedPtr msg) {
          active_services_str = msg->data;
        });

    RCLCPP_INFO(this->get_logger(), "Creado subscriptor para servicios del "
                                    "state_manager /unitree/active_services");

    // Clientes para llamar a state_manager
    sport_mode_client =
        this->create_client<std_srvs::srv::SetBool>("/unitree/sport_mode");

    RCLCPP_INFO(this->get_logger(), "Creado cliente para activar desactivar "
                                    "sport_mode /unitree/sport_mode");

    damp_client = this->create_client<std_srvs::srv::SetBool>("/unitree/damp");

    RCLCPP_INFO(this->get_logger(),
                "Creado cliente para activar modo damp /unitree/damp");

    down_client =
        this->create_client<std_srvs::srv::SetBool>("/unitree/stand_down");

    RCLCPP_INFO(this->get_logger(),
                "Creado cliente para bajar /unitree/stand_down");

    cmd_puber = this->create_publisher<unitree_go::msg::LowCmd>("/lowcmd", 10);

    RCLCPP_INFO(this->get_logger(), "Creado publisher low cmd");

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(int(dt * 1000)),
        std::bind(&low_level_cmd_sender::timer_callback, this));

    // En el constructor, reemplaza las llamadas directas por:
    init_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&low_level_cmd_sender::init_sequence, this));
  }

private:
  enum class State {
    IDLE,
    STAND_UP,
    STAND_DOWN,
    WAIT,
    SETUP_REAR,
    SETUP_FRONT,
    KICKING_GO,
    KICKING_PREPARE,
    FINAL_POSITION,
  };

  double dt = 0.002, runing_time = 0.0, phase = 0.0;
  bool safe_to_move = false;
  bool first_run = true, end = false;
  std::string active_services_str = "";

  State current_state = State::STAND_UP;
  int next = 0;
  double state_start_time = 0.0;

  int log = 0;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr init_timer_;
  rclcpp::Publisher<unitree_go::msg::LowCmd>::SharedPtr cmd_puber;
  rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr state_suber;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr services_suber;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr sport_mode_client;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr damp_client;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr down_client;

  unitree_go::msg::LowCmd low_cmd;
  unitree_go::msg::LowState low_state;

  double initial_joint_pos[12] = {0};

  double stand_up_joint_pos[12] = {
      // FR
      0.00571868,
      0.608813,
      -1.21763,
      // FL
      -0.00571868,
      0.608813,
      -1.21763,
      // RR
      0.00571868,
      0.608813,
      -1.21763,
      // RL
      -0.00571868,
      0.608813,
      -1.21763,
  };

  double experimental_kick_joint_pos[12] = {
      // FR
      0.0473455,
      1.22187,
      -2.44375,
      // FL
      -0.1971,
      0.5587,
      -0.9260,
      // RR
      -0.2305,
      0.886,
      -1.6125,
      // RL
      -0.20427,
      0.965,
      -1.83890,
  };

  double kicking_prepare_joint_pos[12] = {
      // FR
      -0.25,
      1.5,
      -1.2,
      // FL
      -0.1971,
      0.5587,
      -0.9260,
      // RR
      -0.2305,
      0.886,
      -1.6125,
      // RL
      -0.20427,
      0.965,
      -1.83890,
  };

  double kicking_go_joint_pos[12] = {
      // FR
      -0.25,
      -1.5,
      -1.2,
      // FL
      -0.1971,
      0.5587,
      -0.9260,
      // RR
      -0.2305,
      0.886,
      -1.6125,
      // RL
      -0.20427,
      0.965,
      -1.83890,
  };

  double estiralapata[12] = {
      // FR
      0.00571868,
      0.608813,
      -1.21763,
      // FL
      -0.1971,
      0.5587,
      -0.9260,
      // RR
      -0.2305,
      0.886,
      -1.6125,
      // RL
      -0.20427,
      0.965,
      -1.83890,
  };

  double stand_down_joint_pos[12] = {
      0.0473455, 1.22187, -2.44375, -0.0473455, 1.22187, -2.44375,
      0.0473455, 1.22187, -2.44375, -0.0473455, 1.22187, -2.44375};

  //
  // 0-2: FR
  // 3-5: FL
  // 6-8: RR
  // 9-11: RL
  //
  // [0 3 6 9]: Hip (Abducción) ---> RANGO: -1.04 a 1.04 (afuera/adentro)
  // [1 4 7 10]: Thigh (Muslo) ----> RANGO: -1.57 a 3.49 (adelante/atrás)
  // [2 5 8 11]: Calf (Gemelo) ----> RANGO: -2.7 a -0.8 (encojida/estirada)
  //

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

  bool call_down() {
    // Bucle infinito que espera hasta que el servicio aparezca
    while (!down_client->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(),
                     "Nodo interrumpido mientras esperaba al state_manager.");
        return false;
      }
      RCLCPP_INFO(this->get_logger(),
                  "state_manager no disponible para stand_down, esperando...");
    }

    auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
    req->data = true;

    auto future = down_client->async_send_request(req);
    try {
      auto res = future.get();
      RCLCPP_INFO(this->get_logger(), "Stand_down: %s", res->message.c_str());
      return res->success;
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Excepción al llamar stand_down: %s",
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
    RCLCPP_INFO(
        this->get_logger(),
        "Iniciando secuencia de finalizacion activar sport_mode + damp");
    std::thread([this]() {
      // Espera a que sport_mode responda antes de continuar
      if (call_sport_mode(true)) {
        // Solo llama a damp si sport_mode se activó con éxito
        RCLCPP_INFO(this->get_logger(), "Activado el sport_mode");
        RCLCPP_INFO(this->get_logger(), "Llamando a damp!!!");
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

  void change_joints(double joint_arr_dest[12], double joint_arr_org[12],
                     double phase) {
    for (int i = 0; i < 12; i++) {
      low_cmd.motor_cmd[i].q =
          phase * joint_arr_dest[i] + (1 - phase) * joint_arr_org[i];
      low_cmd.motor_cmd[i].dq = 0;
      low_cmd.motor_cmd[i].kp = phase * 60.0 + (1 - phase) * 20.0;
      low_cmd.motor_cmd[i].kd = 5;
      low_cmd.motor_cmd[i].tau = 0;
    }
  }

  void change_joints_rear(double joint_arr_dest[12], double joint_arr_org[12],
                          double phase) {
    for (int i = 6; i < 12; i++) {
      low_cmd.motor_cmd[i].q =
          phase * joint_arr_dest[i] + (1 - phase) * joint_arr_org[i];
      low_cmd.motor_cmd[i].dq = 0;
      low_cmd.motor_cmd[i].kp = phase * 60.0 + (1 - phase) * 20.0;
      low_cmd.motor_cmd[i].kd = 5;
      low_cmd.motor_cmd[i].tau = 0;
    }
  }

  void change_joints_front(double joint_arr_dest[12], double joint_arr_org[12],
                           double phase) {
    for (int i = 0; i < 6; i++) {
      low_cmd.motor_cmd[i].q =
          phase * joint_arr_dest[i] + (1 - phase) * joint_arr_org[i];
      low_cmd.motor_cmd[i].dq = 0;
      low_cmd.motor_cmd[i].kp = phase * 60.0 + (1 - phase) * 20.0;
      low_cmd.motor_cmd[i].kd = 5;
      low_cmd.motor_cmd[i].tau = 0;
    }
  }

  void change_joints_kick(double joint_arr_dest[12], double joint_arr_org[12],
                          double phase) {
    for (int i = 1; i < 2; i++) {
      low_cmd.motor_cmd[i].q =
          phase * joint_arr_dest[i] + (1 - phase) * joint_arr_org[i];
      low_cmd.motor_cmd[i].dq = 0;
      low_cmd.motor_cmd[i].kp = phase * 60.0 + (1 - phase) * 20.0;
      low_cmd.motor_cmd[i].kd = 5;
      low_cmd.motor_cmd[i].tau = 0;
    }
  }

  void change_joints_FR(double joint_arr_dest[12], double joint_arr_org[12],
                        double phase) {
    for (int i = 0; i < 3; i++) {
      low_cmd.motor_cmd[i].q =
          phase * joint_arr_dest[i] + (1 - phase) * joint_arr_org[i];
      low_cmd.motor_cmd[i].dq = 0;
      low_cmd.motor_cmd[i].kp = phase * 60.0 + (1 - phase) * 20.0;
      low_cmd.motor_cmd[i].kd = 5;
      low_cmd.motor_cmd[i].tau = 0;
    }
  }

  void init_sequence() {
    init_timer_->cancel(); // disparo único

    std::thread([this]() {
      RCLCPP_INFO(this->get_logger(), "Esperando a stand_down...");
      if (call_down()) {
        RCLCPP_INFO(this->get_logger(), "Bajado");
      }

      RCLCPP_INFO(this->get_logger(), "Desactivando sport_mode...");
      if (call_sport_mode(false)) {
        safe_to_move = true;
        RCLCPP_INFO(this->get_logger(), "Listo. Comenzando movimiento.");
      }

      init_cmd();
    }).detach();
  }

  void timer_callback() {
    if (!safe_to_move)
      return;
    if (first_run || end)
      return;

    runing_time += dt;
    double state_time = runing_time - state_start_time;

    switch (current_state) {

    case State::STAND_UP: {
      double duration = 3.0;
      phase = tanh(state_time / 1.2);

      if (next) {
        if (log == 6) {
          RCLCPP_INFO(this->get_logger(),
                      "Estirando/Bajando pata FR para "
                      "gracefully down/recuperar equilibro");
          log++;
        }
        change_joints_FR(estiralapata, experimental_kick_joint_pos, phase);
      } else {
        if (log == 0) {
          RCLCPP_INFO(
              this->get_logger(),
              "Levantando robot desde posicion inicial a posicion stand_up");
          log++;
        }
        change_joints(stand_up_joint_pos, initial_joint_pos, phase);
      }

      if (state_time >= duration) {
        if (next) {
          current_state = State::STAND_DOWN;
        } else {
          current_state = State::SETUP_REAR;
        }
        state_start_time = runing_time;
      }
      break;
    }

    case State::SETUP_REAR: {
      double duration = 3.0;
      phase = tanh(state_time / 1.2);

      if (log == 1) {
        RCLCPP_INFO(this->get_logger(),
                    "Colocando parte trasera del robot en posicion estable");
        log++;
      }
      change_joints_rear(experimental_kick_joint_pos, stand_up_joint_pos,
                         phase);

      if (state_time >= duration) {
        current_state = State::SETUP_FRONT;
        state_start_time = runing_time;
      }
      break;
    }

    case State::SETUP_FRONT: {
      double duration = 3.0;
      phase = tanh(state_time / 1.2);

      if (next) {
        if (log == 5) {
          RCLCPP_INFO(this->get_logger(),
                      "Recolocando parte delantera para poder bajar la pata");
          log++;
        }
        change_joints_front(experimental_kick_joint_pos, kicking_go_joint_pos,
                            phase);
      } else {
        if (log == 2) {
          RCLCPP_INFO(
              this->get_logger(),
              "Colocando parte delantera del robot en posicion estable");
          log++;
        }
        change_joints_front(experimental_kick_joint_pos, stand_up_joint_pos,
                            phase);
      }

      if (state_time >= duration) {
        if (next) {
          current_state = State::STAND_UP;
        } else {
          current_state = State::KICKING_PREPARE;
        }
        state_start_time = runing_time;
      }
      break;
    }

    case State::KICKING_PREPARE: {
      double duration = 3.0;
      phase = tanh(state_time / 1.2);

      if (log == 3) {
        RCLCPP_INFO(this->get_logger(), "Preparando accion de patada");
        log++;
      }
      change_joints_front(kicking_prepare_joint_pos,
                          experimental_kick_joint_pos, phase);

      if (state_time >= duration) {
        current_state = State::KICKING_GO;
        state_start_time = runing_time;
      }
      break;
    }

    case State::KICKING_GO: {
      double duration = 3.0;
      phase = tanh(state_time / 1.2);

      if (log == 4) {
        RCLCPP_INFO(this->get_logger(), "Ejecutando patada");
        log++;
      }
      change_joints_kick(kicking_go_joint_pos, experimental_kick_joint_pos,
                         phase);
      next++;

      if (state_time >= duration) {
        current_state = State::SETUP_FRONT;
        state_start_time = runing_time;
      }
      break;
    }

    case State::STAND_DOWN: {
      double duration = 3.0;
      phase = tanh(state_time / 1.2);

      if (log == 7) {
        RCLCPP_INFO(this->get_logger(), "Bajando a posicion stand_down desde "
                                        "estado semi-stand_up (pata estirada)");
        log++;
      }

      change_joints(stand_down_joint_pos, estiralapata, phase);

      if (state_time >= duration) {
        current_state = State::FINAL_POSITION;
        state_start_time = runing_time;
      }
      break;
    }

    case State::FINAL_POSITION: {
      double duration = 4.0;
      double phase = tanh(state_time / 1.8);

      if (log == 8) {
        RCLCPP_INFO(this->get_logger(), "Volviendo abajo del todo");
        log++;
      }

      change_joints(initial_joint_pos, stand_down_joint_pos, phase); // si esta de pie inicialmente vuelve a intentar estar de pie xd meter un array con valores para descansando

      if (state_time >= duration) {
        current_state = State::IDLE;
        state_start_time = runing_time;
      }
      break;
    }

    case State::WAIT: {
      // do nothing, last position
      double duration = 5.0;

      printf("WAIT %f\n", state_time);

      if (state_time >= duration) {
        current_state = State::IDLE;
        state_start_time = runing_time;
      }
      break;
    }

    case State::IDLE: {
      // do nothing, last position
      printf("IDLE\n");
      break;
    }

    default:
      break;
    }

    get_crc(low_cmd);            // Check motor cmd crc
    cmd_puber->publish(low_cmd); // Publish lowcmd message
    if (current_state == State::IDLE) {
      RCLCPP_INFO(this->get_logger(), "Comportamiento terminado");
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
};

int main(int argc, char **argv) {
  std::cout << "Press enter to start";
  std::cin.get();
  rclcpp::init(argc, argv); // sin ChannelFactory aquí
  rclcpp::spin(std::make_shared<low_level_cmd_sender>());
  rclcpp::shutdown();
  return 0;
}
