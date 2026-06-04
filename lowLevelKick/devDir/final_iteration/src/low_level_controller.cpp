#include "motor_crc.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "unitree_go/msg/low_cmd.hpp"
#include "unitree_go/msg/low_state.hpp"

class low_level_cmd_sender : public rclcpp::Node {
public:
  low_level_cmd_sender(bool simulation = false)
      : Node("low_level_cmd_sender"), simulation_(simulation) {

    if (!simulation_) {

      // Suscribirse al estado de servicios publicado por state_manager
      services_suber = this->create_subscription<std_msgs::msg::String>(
          "/unitree/active_services", 10,
          [this](std_msgs::msg::String::SharedPtr msg) {
            active_services_str = msg->data;
          });

      RCLCPP_INFO(this->get_logger(), "Creado subscriptor para servicios del "
                                      "state_manager /unitree/active_services");

      motion_switcher_client = this->create_client<std_srvs::srv::SetBool>(
          "/unitree/motion_switcher");

      RCLCPP_INFO(this->get_logger(),
                  "Creado cliente para activar desactivar "
                  "motion_switcher /unitree/motion_switcher");

      damp_client =
          this->create_client<std_srvs::srv::SetBool>("/unitree/damp");

      RCLCPP_INFO(this->get_logger(),
                  "Creado cliente para activar modo damp /unitree/damp");

      down_client =
          this->create_client<std_srvs::srv::SetBool>("/unitree/stand_down");

      RCLCPP_INFO(this->get_logger(),
                  "Creado cliente para bajar /unitree/stand_down");
    } else {
      init_cmd();
      safe_to_move = true;
      first_run = false;
      for (int i = 0; i < 12; i++)
        initial_joint_pos[i] = stand_down_joint_pos[i];
    }

    state_suber = this->create_subscription<unitree_go::msg::LowState>(
        "/lowstate", 10,
        std::bind(&low_level_cmd_sender::lowstate_callback, this,
                  std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Creado subscriptor /lowstate");

    cmd_puber = this->create_publisher<unitree_go::msg::LowCmd>("/lowcmd", 10);

    RCLCPP_INFO(this->get_logger(), "Creado publisher low cmd");

    // cuidado esto talvez eliminar
    MotionStep stand_up_inicial{.type = MoveType::ALL,
                                .dest = stand_up_joint_pos,
                                .src = initial_joint_pos,
                                .duration = 5.0,
                                .tanh_scale = 1.0,
                                .log_msg = "Levantando robot a stand_up"};

    MotionStep ajuste_patas_traseras{
        .type = MoveType::REAR,
        .dest = experimental_kick_joint_pos,
        .src = stand_up_joint_pos,
        .duration = 3.0,
        .tanh_scale = 1.2,
        .log_msg = "Colocando parte trasera en posicion estable"};

    MotionStep ajuste_patas_delanteras{
        .type = MoveType::FRONT,
        .dest = experimental_kick_joint_pos,
        .src = stand_up_joint_pos,
        .duration = 3.0,
        .tanh_scale = 1.2,
        .log_msg = "Colocando parte delantera en posicion estable"};

    MotionStep preparando_patada{.type = MoveType::FRONT,
                                 .dest = kicking_prepare_joint_pos,
                                 .src = experimental_kick_joint_pos,
                                 .duration = 3.0,
                                 .tanh_scale = 1.2,
                                 .log_msg = "Preparando accion de patada"};

    // para meterle un ostion a la pelota tocar duration y tanh_scale del estado
    // de golpeo a menor tiempo y menor tanh_scale mayor velocidad

    MotionStep golpeo{.type = MoveType::KICK,
                      .dest = kicking_go_joint_pos,
                      .src = experimental_kick_joint_pos,
                      .duration = 0.5,
                      .tanh_scale = 0.2,
                      .log_msg = "Ejecutando patada"};

    MotionStep recolocacion_bajada{
        .type = MoveType::FRONT,
        .dest = experimental_kick_joint_pos,
        .src = kicking_go_joint_pos,
        .duration = 3.0,
        .tanh_scale = 1.2,
        .log_msg = "Recolocando parte delantera para bajar pata"};

    MotionStep bajada_estable{
        .type = MoveType::FR,
        .dest = estiralapata,
        .src = experimental_kick_joint_pos,
        .duration = 3.0,
        .tanh_scale = 1.2,
        .log_msg = "Estirando/Bajando pata FR para recuperar equilibrio"};

    MotionStep stand_down{.type = MoveType::ALL,
                          .dest = stand_down_joint_pos,
                          .src = estiralapata,
                          .duration = 3.0,
                          .tanh_scale = 1.2,
                          .log_msg = "Bajando a posicion stand_down"};

    MotionStep posicion_final{.type = MoveType::ALL,
                              .dest = reposo_joint_pos,
                              .src = stand_down_joint_pos,
                              .duration = 4.0,
                              .tanh_scale = 1.8,
                              .log_msg = "Volviendo a posicion de reposo"};

    steps = {stand_up_inicial,
             ajuste_patas_traseras,
             ajuste_patas_delanteras,
             preparando_patada,
             golpeo,
             recolocacion_bajada,
             bajada_estable,
             stand_down,
             posicion_final};

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(int(dt * 1000)),
        std::bind(&low_level_cmd_sender::timer_callback, this));

    if (!simulation_)
      init_timer_ = this->create_wall_timer(
          std::chrono::milliseconds(100),
          std::bind(&low_level_cmd_sender::init_sequence, this));
  }

private:
  double initial_joint_pos[12] = {0};

  double reposo_joint_pos[12] = {
      // FR
      0.035205,
      1.299193,
      -2.829072,
      // FL
      -0.051343,
      1.291139,
      -2.822169,
      // RR
      -0.336699,
      1.295615,
      -2.801775,
      // RL
      0.314021,
      1.311298,
      -2.805124,
  };

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
      -1.3,
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

  // van en conjunto el 0 de kicking_prepare y go:

  // -0.35 pelota hacia la izq
  // -0.25 recto
  // -0.15 pelota hacia la derecha

  double kicking_go_joint_pos[12] = {
      // FR
      -0.25,
      -1.5,
      -1.3,
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

  void change_joints(double joint_arr_dest[12], double joint_arr_org[12],
                     double phase) {
    for (int i = 0; i < 12; i++) {
      low_cmd.motor_cmd[i].mode = 0x01;
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
      low_cmd.motor_cmd[i].mode = 0x01;
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
      low_cmd.motor_cmd[i].mode = 0x01;
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
      low_cmd.motor_cmd[i].mode = 0x01;
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
      low_cmd.motor_cmd[i].mode = 0x01;
      low_cmd.motor_cmd[i].q =
          phase * joint_arr_dest[i] + (1 - phase) * joint_arr_org[i];
      low_cmd.motor_cmd[i].dq = 0;
      low_cmd.motor_cmd[i].kp = phase * 60.0 + (1 - phase) * 20.0;
      low_cmd.motor_cmd[i].kd = 5;
      low_cmd.motor_cmd[i].tau = 0;
    }
  }

  enum class MoveType {
    ALL,   // change_joints
    REAR,  // change_joints_rear    6-11 (patas traseras)
    FRONT, // change_joints_front   0-5  (patas delanteras)
    KICK,  // change_joints_kick    1    (muslo FR)
    FR,    // change_joints_FR      0-2  (pata FR completa)
  };

  struct MotionStep {
    MoveType type;
    double *dest;
    double *src;
    double duration;
    double tanh_scale;
    std::string log_msg;
  };

  double dt = 0.002, runing_time = 0.0;
  bool simulation_ = false;
  bool safe_to_move = false;
  bool first_run = true, end = false;
  double state_start_time = 0.0;

  std::vector<MotionStep> steps;
  int current_step = 0;
  int last_logged_step = -1;

  std::string active_services_str = "";

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr init_timer_;
  rclcpp::Publisher<unitree_go::msg::LowCmd>::SharedPtr cmd_puber;
  rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr state_suber;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr services_suber;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr motion_switcher_client;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr damp_client;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr down_client;

  unitree_go::msg::LowCmd low_cmd;
  unitree_go::msg::LowState low_state;

  bool call_motion_switcher(bool enable) {
    // Bucle infinito que espera hasta que el servicio aparezca
    while (!motion_switcher_client->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(
            this->get_logger(),
            "Nodo interrumpido mientras esperaba al MotionSwitcherClient.");
        return false;
      }
      RCLCPP_INFO(
          this->get_logger(),
          "MotionSwitcherClient no disponible, esperando a que se encienda...");
    }

    auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
    req->data = enable;

    auto future = motion_switcher_client->async_send_request(req);
    try {
      auto res = future.get();
      RCLCPP_INFO(this->get_logger(), "motion_switcher(%d): %s", enable,
                  res->message.c_str());
      return res->success;
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(),
                   "Excepción al llamar motion_switcher: %s", e.what());
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

  // Mantener solo si interesa que se baje al acabar
  void finish_sequence() {

    if (simulation_) {
      rclcpp::shutdown();
      return;
    }

    RCLCPP_INFO(this->get_logger(),
                "Iniciando secuencia de finalizacion down + damp");
    std::thread([this]() {
      if (call_motion_switcher(false)) {
        RCLCPP_INFO(this->get_logger(), "motion_switcher desactivado");
        RCLCPP_INFO(this->get_logger(), "Bajando y poniendo en damp");
        call_down();
        call_damp();
      } else {
        RCLCPP_ERROR(this->get_logger(), "Fallo, cancelando todo?.");
      }
      rclcpp::shutdown(); // se algo falla a tomar polo cu?
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

  void init_sequence() {
    init_timer_->cancel(); // disparo único

    std::thread([this]() {
      RCLCPP_INFO(this->get_logger(), "Esperando a stand_down...");
      if (call_down()) {
        RCLCPP_INFO(this->get_logger(), "Bajado");
      }

      std::this_thread::sleep_for(std::chrono::seconds(4));

      init_cmd();

      RCLCPP_INFO(
          this->get_logger(),
          "Activando motion_switcher para permitir control bajo nivel...");

      call_motion_switcher(true);

      RCLCPP_INFO(this->get_logger(),
                  "Verificando inicio del motion_switcher...");
      bool verificado = false;

      for (int i = 0; i < 20; i++) { // Intenta durante 10 segundos
        // Comprobamos si la palabra "mcf" ya está en el string init_sequence
        // publica el StateManager
        if (!active_services_str.empty() &&
            active_services_str.find("mcf") != std::string::npos) {
          verificado = true;
          break;
        }
        RCLCPP_INFO(this->get_logger(), "esperando...");
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
      }

      if (verificado) {
        RCLCPP_INFO(this->get_logger(), "motion_switcher configurado.");
        safe_to_move =
            true; // Ahora sí es 100% seguro arrancar el timer de bajo nivel
      } else {
        RCLCPP_ERROR(this->get_logger(),
                     "ERROR al activar el motion_switcher al inicio.");
        safe_to_move = false;
      }
    }).detach();
  }

  void dispatch(const MotionStep &step, double phase) {
    switch (step.type) {
    case MoveType::ALL:
      change_joints(step.dest, step.src, phase);
      break;
    case MoveType::REAR:
      change_joints_rear(step.dest, step.src, phase);
      break;
    case MoveType::FRONT:
      change_joints_front(step.dest, step.src, phase);
      break;
    case MoveType::KICK:
      change_joints_kick(step.dest, step.src, phase);
      break;
    case MoveType::FR:
      change_joints_FR(step.dest, step.src, phase);
      break;
    }
  }

  void timer_callback() {
    if (!safe_to_move || first_run || end)
      return;

    runing_time += dt;
    double state_time = runing_time - state_start_time;

    if (current_step >= (int)steps.size()) {
      if (!end) {
        RCLCPP_INFO(this->get_logger(), "Comportamiento terminado");
        end = true;
        finish_sequence();
      }
      return;
    }

    const auto &step = steps[current_step];
    double phase = tanh(state_time / step.tanh_scale);

    if (last_logged_step != current_step) {
      RCLCPP_INFO(this->get_logger(), "%s", step.log_msg.c_str());
      last_logged_step = current_step;
    }

    dispatch(step, phase);

    if (state_time >= step.duration) {
      current_step++;
      state_start_time = runing_time;
    }

    get_crc(low_cmd);
    cmd_puber->publish(low_cmd);
  }

  void init_cmd() {

    low_cmd.head[0] = 0xFE;
    low_cmd.head[1] = 0xEF;
    low_cmd.level_flag = 0xFF;
    low_cmd.bandwidth = 0x01;

    for (int i = 0; i < 12; i++) {
      low_cmd.motor_cmd[i].mode = 0x01;
      low_cmd.motor_cmd[i].q = PosStopF;
      low_cmd.motor_cmd[i].kp = 0;
      low_cmd.motor_cmd[i].dq = VelStopF;
      low_cmd.motor_cmd[i].kd = 0;
      low_cmd.motor_cmd[i].tau = 0;
    }

    for (int i = 12; i < 20; i++) {
      low_cmd.motor_cmd[i].mode = 0x00;
      low_cmd.motor_cmd[i].q = 0;
      low_cmd.motor_cmd[i].kp = 0;
      low_cmd.motor_cmd[i].dq = 0;
      low_cmd.motor_cmd[i].kd = 0;
      low_cmd.motor_cmd[i].tau = 0;
    }
  }
};

int main(int argc, char **argv) {
  bool sim = (argc > 1 && std::string(argv[1]) == "--sim");
  std::cout << "Press enter to start";
  std::cin.get();
  rclcpp::init(argc, argv); // sin ChannelFactory aquí
  rclcpp::spin(std::make_shared<low_level_cmd_sender>(sim));
  rclcpp::shutdown();
  return 0;
}
