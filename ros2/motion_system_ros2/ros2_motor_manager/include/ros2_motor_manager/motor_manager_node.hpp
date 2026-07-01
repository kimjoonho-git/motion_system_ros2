#ifndef MOTOR_MANAGER_NODE_HPP_
#define MOTOR_MANAGER_NODE_HPP_

#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "driver_interface/msg/ethercat_status.hpp"
#include "motor_status_msgs/msg/motor_status.hpp"
#include "std_msgs/msg/empty.hpp"

#include "motor_manager/motor_manager.hpp"

class MotorManagerNode : public rclcpp::Node {
public:
    using EthercatStatus = driver_interface::msg::EthercatStatus;
    using MotorStatus = motor_status_msgs::msg::MotorStatus;
    using Empty = std_msgs::msg::Empty;

    explicit MotorManagerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    ~MotorManagerNode();

private:
    void motor_command_callback(const MotorStatus::SharedPtr msg);

    void user_command_callback(const Empty::SharedPtr msg);

    void timer_callback();

    void ethercat_status_timer_callback();

    rclcpp::Subscription<MotorStatus>::SharedPtr motor_command_subscriber_;

    rclcpp::Subscription<Empty>::SharedPtr user_command_subscriber_;

    rclcpp::Publisher<MotorStatus>::SharedPtr motor_status_publisher_;

    rclcpp::Publisher<EthercatStatus>::SharedPtr ethercat_status_publisher_;

    rclcpp::TimerBase::SharedPtr motor_status_timer_;

    rclcpp::TimerBase::SharedPtr ethercat_status_timer_;

    std::string config_file_;

    bool auto_enable_{false};

    std::unique_ptr<motor_manager::MotorManager> motor_manager_;

    std::thread manager_run_thread_;
};

#endif // MOTOR_MANAGER_NODE_HPP_
