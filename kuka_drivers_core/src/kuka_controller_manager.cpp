#include "kuka_drivers_core/kuka_controller_manager.h"

#include <memory>

#include "rclcpp/rclcpp.hpp"

namespace kuka_drivers_core {
KukaControllerManager::KukaControllerManager(const rclcpp::NodeOptions&)
    : is_configured_(false) {
  controller_manager_ = std::make_shared<controller_manager::ControllerManager>(
      nullptr, kNodeName);
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
  is_configured_sub_ =
      controller_manager_->create_subscription<std_msgs::msg::Bool>(
          "robot_manager/is_configured", qos,
          [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
            is_configured_.store(msg->data);
          });

  controller_loop_thread_ =
      std::thread(&KukaControllerManager::ControllerMainLoop, this);
  RCLCPP_ERROR_STREAM(controller_manager_->get_logger(),
                      controller_manager_->get_name() << " started");
}

KukaControllerManager::~KukaControllerManager() {
  if (controller_manager_spin_thread_.joinable()) {
    controller_manager_spin_thread_.join();
  }
  if (controller_loop_thread_.joinable()) {
    controller_loop_thread_.join();
  }
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
KukaControllerManager::get_node_base_interface() const {
  return controller_manager_->get_node_base_interface();
}

void KukaControllerManager::ControllerMainLoop() {
  struct sched_param param;
  param.sched_priority = 95;
  if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
    RCLCPP_WARN_STREAM(
        controller_manager_->get_logger(),
        "setscheduler error: "
            << strerror(errno)
            << " - You can use the driver but scheduler priority was not set");
  }

  const rclcpp::Duration dt = rclcpp::Duration::from_seconds(
      1.0 / controller_manager_->get_update_rate());
  std::chrono::milliseconds dt_ms{1000 /
                                  controller_manager_->get_update_rate()};

  try {
    while (rclcpp::ok()) {
      if (is_configured_.load()) {
        controller_manager_->read(controller_manager_->now(), dt);
        controller_manager_->update(controller_manager_->now(), dt);
        controller_manager_->write(controller_manager_->now(), dt);
      } else {
        controller_manager_->update(controller_manager_->now(), dt);
        std::this_thread::sleep_for(dt_ms);
      }
    }
  } catch (std::exception& e) {
    RCLCPP_ERROR(controller_manager_->get_logger(),
                 "Quitting control loop due to: %s", e.what());
  }
}
}  // namespace kuka_drivers_core

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(kuka_drivers_core::KukaControllerManager)
