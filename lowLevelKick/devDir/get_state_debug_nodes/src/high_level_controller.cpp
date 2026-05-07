#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "unitree/robot/channel/channel_factory.hpp"
#include "unitree/robot/go2/robot_state/robot_state_client.hpp"
#include "unitree/robot/go2/sport/sport_client.hpp"

// Servicios a apagar/encender junto con sport_mode
static const std::vector<std::string> MOTION_SERVICES = {"advanced_sport"};

class StateManagerNode : public rclcpp::Node {
public:
  StateManagerNode() : Node("state_manager") {
    client_.SetTimeout(10.0f);
    client_.Init();
    sport_client_.SetTimeout(10.0f);
    sport_client_.Init();

    state_pub_ = this->create_publisher<std_msgs::msg::String>(
        "/unitree/active_services", 10);

    sport_mode_srv_ = this->create_service<std_srvs::srv::SetBool>(
        "/unitree/sport_mode",
        std::bind(&StateManagerNode::sport_mode_callback, this,
                  std::placeholders::_1, std::placeholders::_2));

    damp_srv_ = this->create_service<std_srvs::srv::SetBool>(
        "/unitree/damp",
        std::bind(&StateManagerNode::damp_callback, this, std::placeholders::_1,
                  std::placeholders::_2));

    stand_up_srv_ = this->create_service<std_srvs::srv::SetBool>(
        "/unitree/stand_up",
        std::bind(&StateManagerNode::stand_up_callback, this,
                  std::placeholders::_1, std::placeholders::_2));

    stand_down_srv_ = this->create_service<std_srvs::srv::SetBool>(
        "/unitree/stand_down",
        std::bind(&StateManagerNode::stand_down_callback, this,
                  std::placeholders::_1, std::placeholders::_2));

    balance_stand_srv_ = this->create_service<std_srvs::srv::SetBool>(
        "/unitree/balance_stand",
        std::bind(&StateManagerNode::balance_stand_callback, this,
                  std::placeholders::_1, std::placeholders::_2));

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&StateManagerNode::publish_state, this));

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

  // Apaga o enciende todos los servicios de movimiento
  bool switch_all_motion_services(bool enable) {
    bool all_ok = true;
    for (const auto &svc : MOTION_SERVICES) {
      int32_t status;
      int32_t ret = client_.ServiceSwitch(svc, enable ? 1 : 0, status);
      RCLCPP_INFO(this->get_logger(), "%s %s | ret=%d status=%d",
                  enable ? "Activando" : "Apagando", svc.c_str(), ret, status);
      if (ret != 0)
        all_ok = false;
    }
    return all_ok;
  }

  void sport_mode_callback(const std_srvs::srv::SetBool::Request::SharedPtr req,
                           std_srvs::srv::SetBool::Response::SharedPtr res) {

    bool ok = switch_all_motion_services(req->data);
    res->success = ok;
    res->message = ok ? "OK" : "Algún servicio falló, revisa logs";
  }

  void damp_callback(const std_srvs::srv::SetBool::Request::SharedPtr req,
                     std_srvs::srv::SetBool::Response::SharedPtr res) {

    if (!req->data) {
      res->success = false;
      res->message = "Damp solo acepta true";
      return;
    }
    sport_client_.Damp();
    res->success = true;
    res->message = "Damp ejecutado";
    RCLCPP_INFO(this->get_logger(), "Damp ejecutado.");
  }

  void stand_up_callback(const std_srvs::srv::SetBool::Request::SharedPtr req,
                         std_srvs::srv::SetBool::Response::SharedPtr res) {
    if (!req->data) {
      res->success = false;
      res->message = "StandUp solo acepta true";
      return;
    }
    sport_client_.StandUp();
    res->success = true;
    res->message = "StandUp ejecutado";
    RCLCPP_INFO(this->get_logger(), "StandUp ejecutado.");
  }

  void stand_down_callback(const std_srvs::srv::SetBool::Request::SharedPtr req,
                           std_srvs::srv::SetBool::Response::SharedPtr res) {
    if (!req->data) {
      res->success = false;
      res->message = "StandDown solo acepta true";
      return;
    }
    sport_client_.StandDown();
    res->success = true;
    res->message = "StandDown ejecutado";
    RCLCPP_INFO(this->get_logger(), "StandDown ejecutado.");
  }

  void
  balance_stand_callback(const std_srvs::srv::SetBool::Request::SharedPtr req,
                         std_srvs::srv::SetBool::Response::SharedPtr res) {
    if (!req->data) {
      res->success = false;
      res->message = "Balance Stand solo acepta true";
      return;
    }
    sport_client_.BalanceStand();
    res->success = true;
    res->message = "Balance Stand ejecutado";
    RCLCPP_INFO(this->get_logger(), "Balance Stand ejecutado.");
  }

  unitree::robot::go2::RobotStateClient client_;
  unitree::robot::go2::SportClient sport_client_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr sport_mode_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr damp_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr stand_up_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr stand_down_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr balance_stand_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
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
  if (argc > 1)
    unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
  else
    unitree::robot::ChannelFactory::Instance()->Init(0);

  init_robot_state();
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StateManagerNode>());
  rclcpp::shutdown();
  return 0;
}
