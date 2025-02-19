#ifndef THIRD_PARTY_KUKA_DRIVERS_KUKA_DRIVERS_CORE_INCLUDE_KUKA_DRIVERS_CORE_KUKA_CONTROLLER_MANAGER_H_
#define THIRD_PARTY_KUKA_DRIVERS_KUKA_DRIVERS_CORE_INCLUDE_KUKA_DRIVERS_CORE_KUKA_CONTROLLER_MANAGER_H_

#include <atomic>
#include <memory>

#include "controller_manager/controller_manager.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

namespace kuka_drivers_core {

class KukaControllerManager {
 public:
  static constexpr const char* kNodeName = "kuka_controller_manager";
  explicit KukaControllerManager(const rclcpp::NodeOptions&);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
  get_node_base_interface() const;

  ~KukaControllerManager();

 private:
  std::atomic_bool is_configured_;
  std::shared_ptr<controller_manager::ControllerManager> controller_manager_;
  std::thread controller_manager_spin_thread_;
  std::thread controller_loop_thread_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr is_configured_sub_;

  void ControllerMainLoop();
};
}  // namespace kuka_drivers_core

#endif  // THIRD_PARTY_KUKA_DRIVERS_KUKA_DRIVERS_CORE_INCLUDE_KUKA_DRIVERS_CORE_KUKA_CONTROLLER_MANAGER_H_
