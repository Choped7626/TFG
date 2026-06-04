#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "unitree/robot/b2/motion_switcher/motion_switcher_client.hpp"
#include "unitree/robot/channel/channel_factory.hpp"
#include "unitree/robot/go2/robot_state/robot_state_client.hpp"
#include "unitree/robot/go2/sport/sport_client.hpp"
#include <cstdio>
#include <string>

class MotionSwitcherNode : public rclcpp::Node {
public:
  MotionSwitcherNode() : Node("state_manager") {

    client_.Init();
    sport_client_.Init();
    motion_switcher_client_.Init();

    state_pub_ = this->create_publisher<std_msgs::msg::String>(
        "/unitree/active_services", 10);

    RCLCPP_INFO(this->get_logger(), "Publisher en /unitree/active_services");

    motion_switcher_srv_ = this->create_service<std_srvs::srv::SetBool>(
        "/unitree/motion_switcher",
        std::bind(&MotionSwitcherNode::motion_switcher_callback, this,
                  std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "Servicio en /unitree/motion_switcher");

    damp_srv_ = this->create_service<std_srvs::srv::SetBool>(
        "/unitree/damp",
        std::bind(&MotionSwitcherNode::damp_callback, this,
                  std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "Servicio en /unitree/damp");

    stand_up_srv_ = this->create_service<std_srvs::srv::SetBool>(
        "/unitree/stand_up",
        std::bind(&MotionSwitcherNode::stand_up_callback, this,
                  std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "Servicio en /unitree/stand_up");

    stand_down_srv_ = this->create_service<std_srvs::srv::SetBool>(
        "/unitree/stand_down",
        std::bind(&MotionSwitcherNode::stand_down_callback, this,
                  std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "Servicio en /unitree/stand_down");

    balance_stand_srv_ = this->create_service<std_srvs::srv::SetBool>(
        "/unitree/balance_stand",
        std::bind(&MotionSwitcherNode::balance_stand_callback, this,
                  std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "Servicio en /unitree/balance_stand");

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&MotionSwitcherNode::publish_state, this));

    RCLCPP_INFO(this->get_logger(), "StateManager listo.");
  }

private:
  void publish_state() {
    std::vector<unitree::robot::go2::ServiceState> services;
    int32_t ret = client_.ServiceList(services);

    if (ret != 0)
      return;

    std::string active = "";
    for (auto &s : services) {
      if (s.status == 1)
        active += s.name + " ";
    }

    auto msg = std_msgs::msg::String();
    msg.data = active;
    state_pub_->publish(msg);
  }

  void
  motion_switcher_callback(const std_srvs::srv::SetBool::Request::SharedPtr req,
                           std_srvs::srv::SetBool::Response::SharedPtr res) {

    std::string form, name;
    int32_t result = -1;
    motion_switcher_client_.CheckMode(form, name);

    if (!req->data) {

      if (name == "mcf") {
        res->success = true;
        res->message = "El MCF ya está activo.";
        RCLCPP_INFO(this->get_logger(), "%s", res->message.c_str());
        return;
      }

      RCLCPP_INFO(this->get_logger(),
                  "Intentando reactivar el MCF...");
      result = motion_switcher_client_.SelectMode("mcf");

      if (result == 0) {
        res->success = true;
        res->message = "MCF activado correctamente.";
      } else {
        res->success = false;
        res->message = "Error al ejecutar SelectMode('mcf'). Código: " +
                       std::to_string(result);
      }
    } else {

      if (name.empty()) {
        res->success = true;
        res->message = "El ReleaseMode ya estaba activado";
        RCLCPP_INFO(this->get_logger(), "%s", res->message.c_str());
        return;
      }

      RCLCPP_INFO(this->get_logger(), "Liberando canales del MCF");
      result = motion_switcher_client_.ReleaseMode();

      if (result == 0) {
        res->success = true;
        res->message = "Canales libres.";
      } else {
        res->success = false;
        res->message =
            "Error al ejecutar ReleaseMode. Código: " + std::to_string(result);
      }
    }
  }

  void damp_callback(const std_srvs::srv::SetBool::Request::SharedPtr req,
                     std_srvs::srv::SetBool::Response::SharedPtr res) {
    if (!req->data) {
      res->success = false;
      res->message = "Damp solo acepta true";
    } else {
      sport_client_.Damp();
      res->success = true;
      res->message = "Damp ejecutado";
      RCLCPP_INFO(this->get_logger(), "Damp ejecutado.");
    }
  }

  void stand_up_callback(const std_srvs::srv::SetBool::Request::SharedPtr req,
                         std_srvs::srv::SetBool::Response::SharedPtr res) {
    if (!req->data) {
      res->success = false;
      res->message = "StandUp solo acepta true";
    } else {
      sport_client_.StandUp();
      res->success = true;
      res->message = "StandUp ejecutado";
      RCLCPP_INFO(this->get_logger(), "StandUp ejecutado.");
    }
  }

  void stand_down_callback(const std_srvs::srv::SetBool::Request::SharedPtr req,
                           std_srvs::srv::SetBool::Response::SharedPtr res) {
    if (!req->data) {
      res->success = false;
      res->message = "StandDown solo acepta true";
    } else {
      sport_client_.StandDown();
      res->success = true;
      res->message = "StandDown ejecutado";
      RCLCPP_INFO(this->get_logger(), "StandDown ejecutado.");
    }
  }

  void
  balance_stand_callback(const std_srvs::srv::SetBool::Request::SharedPtr req,
                         std_srvs::srv::SetBool::Response::SharedPtr res) {
    if (!req->data) {
      res->success = false;
      res->message = "Balance Stand solo acepta true";
    } else {
      sport_client_.BalanceStand();
      res->success = true;
      res->message = "Balance Stand ejecutado";
      RCLCPP_INFO(this->get_logger(), "Balance Stand ejecutado.");
    }
  }

  unitree::robot::go2::RobotStateClient client_;
  unitree::robot::go2::SportClient sport_client_;
  unitree::robot::b2::MotionSwitcherClient motion_switcher_client_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr motion_switcher_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr damp_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr stand_up_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr stand_down_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr balance_stand_srv_;

  rclcpp::TimerBase::SharedPtr timer_;
};

void init_robot_state() {

  unitree::robot::go2::RobotStateClient client;
  client.SetTimeout(5.0f);
  client.Init();

  std::vector<unitree::robot::go2::ServiceState> serviceList;
  int32_t ret = client.ServiceList(serviceList);

  if (ret != 0) {
    printf("Error al obtener servicios: %d", ret);
    return;
  }

  std::cout << "\n--- LISTA DE SERVICIOS DETECTADOS ---" << std::endl;
  printf("%-30s %-10s %-10s\n", "Nombre", "Status", "Protect");

  for (const auto &svc : serviceList) {
    printf("%-30s %-10d %-10d\n", svc.name.c_str(), svc.status, svc.protect);
  }

  std::cout << "-------------------------------------\n" << std::endl;
}

int main(int argc, char **argv) {
  if (argc > 1)
    unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
  else
    unitree::robot::ChannelFactory::Instance()->Init(0);

  init_robot_state();
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotionSwitcherNode>());
  rclcpp::shutdown();
  return 0;
}
