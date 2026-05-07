/**
 * This example demonstrates how to use ROS2 to send low-level motor commands of
 * unitree go2 robot
 **/
#include "motor_crc.h"
#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/bms_cmd.hpp"
#include "unitree_go/msg/low_cmd.hpp"
#include "unitree_go/msg/motor_cmd.hpp"

#include "unitree/robot/channel/channel_factory.hpp"

#include "unitree/robot/go2/robot_state/robot_state_client.hpp"

#include "unitree_go/msg/low_state.hpp"
#include <cstdio>
#include <thread>

// Create a low_level_cmd_sender class for low state receive
class low_level_cmd_sender : public rclcpp::Node {
public:
  low_level_cmd_sender() : Node("low_level_cmd_sender") {

    // init_robot_state();

    state_suber = this->create_subscription<unitree_go::msg::LowState>(
        "/lowstate", 10,
        std::bind(&low_level_cmd_sender::lowstate_callback, this,
                  std::placeholders::_1));

    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    cmd_puber = this->create_publisher<unitree_go::msg::LowCmd>("/lowcmd", 10);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(int(dt * 1000)),
        std::bind(&low_level_cmd_sender::timer_callback, this));

    init_cmd();
  }

private:
  /*

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

  */

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
      for (int i = 0; i < 12; i++) {
        printf("i = %d joint = %lf \n", i, initial_joint_pos[i]);
      }
    }
  }

  void timer_callback() {

    if (first_run)
      return;

    runing_time += dt;
    /*

    if (runing_time < 3.0) {
      // Stand up in first 3 second

      // Total time for standing up or standing down is about 1.2s
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
      // Then stand down
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

    */

    get_crc(low_cmd);            // Check motor cmd crc
    cmd_puber->publish(low_cmd); // Publish lowcmd message
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

  // std::vector<std::string> active_motion_services;
  // unitree::robot::go2::RobotStateClient client;

  rclcpp::TimerBase::SharedPtr timer_; // ROS2 timer
  rclcpp::Publisher<unitree_go::msg::LowCmd>::SharedPtr
      cmd_puber; // ROS2 Publisher
  rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr state_suber;

  unitree_go::msg::LowCmd low_cmd;
  unitree_go::msg::LowState low_state;

  double stand_up_joint_pos[12] = {
      0.00571868, 0.608813, -1.21763, -0.00571868, 0.608813, -1.21763,
      0.00571868, 0.608813, -1.21763, -0.00571868, 0.608813, -1.21763};
  double initial_joint_pos[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

  double dt = 0.002;
  double runing_time = 0.0;
  double phase = 0.0;
  bool first_run = true;
};

void init_robot_state() {

  printf("ESTOY VIVO\n");

  std::vector<std::string> active_motion_services;
  unitree::robot::go2::RobotStateClient client;
  client.SetTimeout(5.0f);
  client.Init();

  std::vector<unitree::robot::go2::ServiceState> serviceList;
  int32_t ret = client.ServiceList(serviceList);

  if (ret != 0) {
    // RCLCPP_ERROR(this->get_logger(), "Error al obtener servicios: %d", ret);
    return;
  }

  std::cout << "\n--- LISTA DE SERVICIOS DETECTADOS ---" << std::endl;
  printf("%-30s %-10s %-10s\n", "Nombre", "Status", "Protect");

  for (const auto &svc : serviceList) {
    printf("%-30s %-10d %-10d\n", svc.name.c_str(), svc.status, svc.protect);

    bool is_motion_service = (svc.name == "mcf" || svc.name == "sport_mode");
    if (is_motion_service && svc.status == 1 && svc.protect == 0) {
      active_motion_services.push_back(svc.name);
      // RCLCPP_INFO(this->get_logger(),
      //             "Servicio de movimiento activo guardado: %s",
      //             svc.name.c_str());
    }
  }
  std::cout << "-------------------------------------\n" << std::endl;

  if (active_motion_services.empty()) {
    // RCLCPP_WARN(this->get_logger(),
    //             "No se detectó ningún servicio de movimiento activo.");
  }
}

int main(int argc, char **argv) {
  std::cout << "Press enter to start";
  std::cin.get();

  // rclcpp::init(argc, argv); // Initialize rclcpp

  if (argc > 1) {
    printf("%s\n", argv[1]);
    unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
  } else {
    unitree::robot::ChannelFactory::Instance()->Init(0); // usa entorno
  }

  init_robot_state();

  // rclcpp::TimerBase::SharedPtr
  //     timer_; // Create a timer callback object to send cmd in time intervals
  // auto node =
  //     std::make_shared<low_level_cmd_sender>(); // Create a ROS2 node and
  //     make
  //  share with
  //  low_level_cmd_sender class
  // rclcpp::spin(node);                           // Run ROS2 node
  // rclcpp::shutdown();                           // Exit
  return 0;
}
