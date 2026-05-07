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
    STAND_DOWN,
    WAIT,
    SETUP_REAR,
    SETUP_FRONT,
    KICKING_GO,
    KICKING_PREPARE,
  };

  State current_state = State::STAND_UP;
  double state_start_time = 0.0;
  double dt = 0.002;
  double runing_time = 0.0;
  double phase = 0.0;

  rclcpp::TimerBase::SharedPtr timer_; // ROS2 timer
  rclcpp::Publisher<unitree_go::msg::LowCmd>::SharedPtr
      cmd_puber; // ROS2 Publisher

  unitree_go::msg::LowCmd low_cmd;

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

  double stand_down_joint_pos[12] = {
      0.0473455, 1.22187, -2.44375, -0.0473455, 1.22187, -2.44375,
      0.0473455, 1.22187, -2.44375, -0.0473455, 1.22187, -2.44375};

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
      -0.0,
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
      -0.8,
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

  double stable_kick_joint_pos[12] = {
      // FR
      0.058142,
      -1.1651,
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

  void timer_callback() {

    runing_time += dt;
    double state_time = runing_time - state_start_time;

    switch (current_state) {

    case State::STAND_UP: {
      double duration = 3.0;
      phase = tanh(state_time / 1.2);

      change_joints(stand_up_joint_pos, stand_down_joint_pos, phase);

      if (state_time >= duration) {
        current_state = State::SETUP_REAR;
        state_start_time = runing_time;
      }
      break;
    }

    case State::SETUP_REAR: {
      double duration = 3.0;
      phase = tanh(state_time / 1.2);

      change_joints_rear(experimental_kick_joint_pos, stand_up_joint_pos, phase);

      if (state_time >= duration) {
        current_state = State::SETUP_FRONT;
        state_start_time = runing_time;
      }
      break;
    }

    case State::SETUP_FRONT: {
      double duration = 3.0;
      phase = tanh(state_time / 1.2);

      change_joints_front(experimental_kick_joint_pos, stand_up_joint_pos, phase);

      if (state_time >= duration) {
        current_state = State::KICKING_PREPARE;
        state_start_time = runing_time;
      }
      break;
    }

    case State::KICKING_PREPARE: {
      double duration = 3.0;
      phase = tanh(state_time / 1.2);

      change_joints_front(kicking_prepare_joint_pos, experimental_kick_joint_pos, phase);

      if (state_time >= duration) {
        current_state = State::KICKING_GO;
        state_start_time = runing_time;
      }
      break;
    }

    case State::KICKING_GO: {
      double duration = 3.0;
      phase = tanh(state_time / 1.2);

      change_joints_kick(kicking_go_joint_pos, experimental_kick_joint_pos, phase);

      if (state_time >= duration) {
        current_state = State::IDLE;
        state_start_time = runing_time;
      }
      break;
    }

    case State::STABLE_FR_IN_AIR: {
      double duration = 4.0;
      double phase = tanh(state_time / 1.8);

      change_joints(stable_kick_joint_pos, stand_up_joint_pos, phase);

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

      if (state_time >= duration){
        current_state = State::IDLE;
	state_start_time = runing_time;
      }
      break;
    }

    case State::STAND_DOWN: {
      double duration = 3.0;
      phase = tanh(state_time / 1.2);

      change_joints(stand_down_joint_pos, stand_up_joint_pos, phase);

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
