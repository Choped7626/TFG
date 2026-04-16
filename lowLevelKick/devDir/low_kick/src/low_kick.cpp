/**
 * This example demonstrates how to use ROS2 to send low-level motor commands of
 * unitree go2 robot
 **/
#include "motor_crc.h"
#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/bms_cmd.hpp"
#include "unitree_go/msg/low_cmd.hpp"
#include "unitree_go/msg/motor_cmd.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <math.h>
#include <stdexcept>

// Create a low_level_cmd_sender class for low state receive
class low_level_cmd_sender : public rclcpp::Node {

public:
  low_level_cmd_sender() : Node("low_level_cmd_sender") {
    // the cmd_puber is set to subscribe "/lowcmd" topic
    cmd_puber = this->create_publisher<unitree_go::msg::LowCmd>("/lowcmd", 10);

    // The timer is set to 200Hz, and bind to
    // low_level_cmd_sender::timer_callback function
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(int(dt * 1000)),
        std::bind(&low_level_cmd_sender::timer_callback, this));

    // Initialize lowcmd
    init_cmd();
  }

private:
  enum class State {
    IDLE,
    STAND_UP,
    STABLE_FR_IN_AIR,
  };

  State current_state = State::STAND_UP;
  double state_start_time = 0.0;

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

  rclcpp::TimerBase::SharedPtr timer_; // ROS2 timer
  rclcpp::Publisher<unitree_go::msg::LowCmd>::SharedPtr
      cmd_puber; // ROS2 Publisher

  unitree_go::msg::LowCmd low_cmd;

  double stand_up_joint_pos[12] = {
      0.00571868, 0.608813, -1.21763, -0.00571868, 0.608813, -1.21763,
      0.00571868, 0.608813, -1.21763, -0.00571868, 0.608813, -1.21763};

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

  double stable_kick_joint_pos[12] = {

      0.01, 0.6,  -2.7,  -0.2,
      0.7,  -1.4,

      -0.2, 0.5,  -1.17, 0.1,
      0.6,  -1.4

  };

  double dt = 0.002;
  double runing_time = 0.0;
  double phase = 0.0;

  void timer_callback() {

    runing_time += dt;
    double state_time = runing_time - state_start_time;

    switch (current_state) {
    case State::STAND_UP: {
      double duration = 3.0;
      double t = std::min(1.0, state_time / duration);
      double phase = 0.5 * (1.0 - cos(t * M_PI));

      for (int i = 0; i < 12; i++) {
        low_cmd.motor_cmd[i].q = phase * stand_up_joint_pos[i] +
                                 (1 - phase) * stand_down_joint_pos[i];
        low_cmd.motor_cmd[i].kp = 40.0;
        low_cmd.motor_cmd[i].kd = 3.0;
      }

      if (t >= 1.0) {
        current_state = State::STABLE_FR_IN_AIR;
        state_start_time = runing_time;
      }
      break;
    }
    case State::STABLE_FR_IN_AIR: {
      double duration = 4.0;
      double t = std::min(1.0, state_time / duration);
      double phase = 0.5 * (1.0 - cos(t * M_PI));

      for (int i = 0; i < 12; i++) {
        low_cmd.motor_cmd[i].q = phase * stand_down_joint_pos[i] +
                                 (1 - phase) * stand_up_joint_pos[i];
        low_cmd.motor_cmd[i].kp = 40.0;
        low_cmd.motor_cmd[i].kd = 3.0;
      }

      if (t >= 1.0) {
        current_state = State::IDLE;
        state_start_time = runing_time;
      }
      break;
    }
    case State::IDLE: {
      double duration = 2.0;
      double t = std::min(1.0, state_time / duration);
      double phase = 0.5 * (1.0 - cos(t * M_PI));

      // for

      printf("IDLE\n");

      if (t >= 1.0) {
        current_state = State::IDLE;
        state_start_time = runing_time;
      }
      break;
    }
    default:
      break;
    }
  }
  /*
    void timer_callback() {

      runing_time += dt;

      if (runing_time < 3.0) {

        double local_phase = tanh(runing_time / 1.2);
        for (int i = 0; i < 12; i++) {
          low_cmd.motor_cmd[i].q = local_phase * stand_up_joint_pos[i] +
                                   (1 - local_phase) * stand_down_joint_pos[i];
          low_cmd.motor_cmd[i].dq = 0;
          low_cmd.motor_cmd[i].kp = local_phase * 50.0 + (1 - local_phase)
    * 20.0; low_cmd.motor_cmd[i].kd = 3.5; low_cmd.motor_cmd[i].tau = 0;
        }

      } else if (runing_time < 4.0) {

      } else if (runing_time < 5.0) {

        double local_phase = tanh((runing_time - 4.0) / 1.0);
        for (int i = 9; i < 12; i++) {
          low_cmd.motor_cmd[i].q = local_phase * stable_kick_joint_pos[i] +
                                   (1 - local_phase) * stand_up_joint_pos[i];
          low_cmd.motor_cmd[i].dq = 0;
          low_cmd.motor_cmd[i].kp = local_phase * 50.0 + (1 - local_phase)
    * 20.0; low_cmd.motor_cmd[i].kd = 3.5; low_cmd.motor_cmd[i].tau = 0;
        }

      } else if (runing_time < 6.0) {

      } else if (runing_time < 7.0) {

        double local_phase = tanh((runing_time - 6.0) / 1.0);
        ;
        for (int i = 3; i < 6; i++) {
          low_cmd.motor_cmd[i].q = local_phase * stable_kick_joint_pos[i] +
                                   (1 - local_phase) * stand_up_joint_pos[i];
          low_cmd.motor_cmd[i].dq = 0;
          low_cmd.motor_cmd[i].kp = local_phase * 50.0 + (1 - local_phase)
    * 20.0; low_cmd.motor_cmd[i].kd = 3.5; low_cmd.motor_cmd[i].tau = 0;
        }

      } else if (runing_time < 8.0) {

      } else if (runing_time < 9.0) {

        double local_phase = tanh((runing_time - 8.0) / 1.0);
        ;
        for (int i = 6; i < 9; i++) {
          low_cmd.motor_cmd[i].q = local_phase * stable_kick_joint_pos[i] +
                                   (1 - local_phase) * stand_up_joint_pos[i];
          low_cmd.motor_cmd[i].dq = 0;
          low_cmd.motor_cmd[i].kp = local_phase * 50.0 + (1 - local_phase)
    * 20.0; low_cmd.motor_cmd[i].kd = 3.5; low_cmd.motor_cmd[i].tau = 0;
        }

      } else if (runing_time < 10.0) {

      } else if (runing_time < 11.0) {

        double local_phase = tanh((runing_time - 10.0) / 1.0);
        ;
        for (int i = 0; i < 3; i++) {
          low_cmd.motor_cmd[i].q = local_phase * stable_kick_joint_pos[i] +
                                   (1 - local_phase) * stand_up_joint_pos[i];
          low_cmd.motor_cmd[i].dq = 0;
          low_cmd.motor_cmd[i].kp = local_phase * 50.0 + (1 - local_phase)
    * 20.0; low_cmd.motor_cmd[i].kd = 3.5; low_cmd.motor_cmd[i].tau = 0;
        }
      }

      get_crc(low_cmd);            // Check motor cmd crc
      cmd_puber->publish(low_cmd); // Publish lowcmd message
    }
    */
};

int main(int argc, char **argv) {
  std::cout << "Press enter to start";
  std::cin.get();

  rclcpp::init(argc, argv); // Initialize rclcpp
  rclcpp::TimerBase::SharedPtr
      timer_; // Create a timer callback object to send cmd in time intervals
  auto node =
      std::make_shared<low_level_cmd_sender>(); // Create a ROS2 node and make
                                                // share with
                                                // low_level_cmd_sender class
  rclcpp::spin(node);                           // Run ROS2 node
  rclcpp::shutdown();                           // Exit
  return 0;
}

