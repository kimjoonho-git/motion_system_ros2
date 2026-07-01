#include <algorithm>
#include <chrono>
#include <memory>
#include <stdexcept>

#include "common_motor_interface/motor_frame.hpp"

#include "ros2_motor_manager/motor_manager_node.hpp"

MotorManagerNode::MotorManagerNode(const rclcpp::NodeOptions& options)
    : Node("motor_manager_node", options)
{
    motor_command_subscriber_ = this->create_subscription<MotorStatus>(
        "motor_command", rclcpp::QoS(1).best_effort(),
        [this](const MotorStatus::SharedPtr msg) {
            motor_command_callback(msg);
        }
    );

    user_command_subscriber_ = this->create_subscription<Empty>(
        "user_command", rclcpp::QoS(1).best_effort(),
        [this](const Empty::SharedPtr msg) {
            user_command_callback(msg);
        }
    );

    motor_status_publisher_ = this->create_publisher<MotorStatus>(
        "motor_status", rclcpp::QoS(1).best_effort()
    );

    ethercat_status_publisher_ = this->create_publisher<EthercatStatus>(
        "ethercat_status", rclcpp::QoS(1).best_effort()
    );

    motor_status_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1),
        [this]() {
            timer_callback();
        }
    );

    ethercat_status_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        [this]() {
            ethercat_status_timer_callback();
        }
    );

    config_file_ = this->declare_parameter<std::string>("config_file", "");
    auto_enable_ = this->declare_parameter<bool>("auto_enable", false);
    if (config_file_.empty()) {
        throw std::runtime_error(
            "Parameter 'config_file' is empty. Use e.g. "
            "`ros2 launch ros2_motor_manager motor_manager.launch.py`.");
    }

    RCLCPP_INFO(
        get_logger(),
        "motor_manager_node starting with auto_enable=%s",
        auto_enable_ ? "true" : "false");

    motor_manager_ = std::make_unique<motor_manager::MotorManager>(config_file_, auto_enable_);

    manager_run_thread_ = std::thread([this]() {
        try {
            motor_manager_->run();
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "MotorManager::run() failed: %s", e.what());
        }
    });
}

MotorManagerNode::~MotorManagerNode()
{
    if (motor_manager_) {
        motor_manager_->request_exit();
    }
    if (manager_run_thread_.joinable()) {
        manager_run_thread_.join();
    }
}

void MotorManagerNode::motor_command_callback(const MotorStatus::SharedPtr msg)
{
    const size_t size = std::min({
        msg->controller_index.size(),
        msg->number_of_target_interfaces.size(),
        msg->target_interface_id.size(),
        msg->controlword.size(),
        msg->statusword.size(),
        msg->errorcode.size(),
        msg->position.size(),
        msg->velocity.size(),
        msg->torque.size(),
        static_cast<size_t>(motor_interface::MAX_CONTROLLER_SIZE),
    });
    //const uint8_t size = motor_manager_->number_of_controllers();
    
    motor_interface::motor_frame_t motor_frame[motor_interface::MAX_CONTROLLER_SIZE] = {};

    for (uint8_t i = 0; i < static_cast<uint8_t>(size); i++) {
        const size_t n_if = std::min({
            static_cast<size_t>(msg->number_of_target_interfaces[i]),
            static_cast<size_t>(motor_interface::MAX_INTERFACE_SIZE),
            msg->target_interface_id[i].data.size(),
        });
        motor_frame[i].number_of_target_interfaces = static_cast<uint8_t>(n_if);

        for (uint8_t j = 0; j < n_if; j++) {
            motor_frame[i].target_interface_id[j] = msg->target_interface_id[i].data[j];
        }
        motor_frame[i].controller_index = msg->controller_index[i];
        motor_frame[i].controlword = msg->controlword[i];
        motor_frame[i].statusword = msg->statusword[i];
        motor_frame[i].errorcode = msg->errorcode[i];
        motor_frame[i].position = msg->position[i];
        motor_frame[i].velocity = msg->velocity[i];
        motor_frame[i].torque = msg->torque[i];
    }

    motor_manager_->write(motor_frame, static_cast<uint8_t>(size));
}

void MotorManagerNode::user_command_callback(const Empty::SharedPtr msg)
{
    (void)msg;

    motor_manager_->request_stop();
}

void MotorManagerNode::timer_callback()
{
    const uint8_t n = motor_manager_->number_of_controllers();
    if (n == 0) {
        return;
    }

    motor_interface::motor_frame_t status[motor_interface::MAX_CONTROLLER_SIZE] = {};
    motor_manager_->read(status);

    MotorStatus msg;
    msg.number_of_target_interfaces.resize(n);
    msg.controller_index.resize(n);
    msg.driver_name.resize(n);
    msg.controlword.resize(n);
    msg.statusword.resize(n);
    msg.errorcode.resize(n);
    msg.station_alias_register.resize(n);
    msg.position.resize(n);
    msg.velocity.resize(n);
    msg.torque.resize(n);
    msg.current.resize(n);
    msg.position_raw.resize(n);
    msg.velocity_raw.resize(n);
    msg.torque_raw.resize(n);
    msg.current_raw.resize(n);

    for (uint8_t i = 0; i < n; i++) {
        msg.controller_index[i] = status[i].controller_index;
        msg.driver_name[i] = status[i].driver_name;
        msg.controlword[i] = status[i].controlword;
        msg.statusword[i] = status[i].statusword;
        msg.errorcode[i] = status[i].errorcode;
        msg.station_alias_register[i] = status[i].station_alias_register;
        msg.position[i] = status[i].position;
        msg.velocity[i] = status[i].velocity;
        msg.torque[i] = status[i].torque;
        msg.current[i] = status[i].current;
        msg.position_raw[i] = status[i].position_raw;
        msg.velocity_raw[i] = status[i].velocity_raw;
        msg.torque_raw[i] = status[i].torque_raw;
        msg.current_raw[i] = status[i].current_raw;
    }

    motor_status_publisher_->publish(msg);
}

void MotorManagerNode::ethercat_status_timer_callback()
{
    if (!motor_manager_) {
        return;
    }

    const auto statuses = motor_manager_->ethercat_statuses();
    if (statuses.empty()) {
        return;
    }

    EthercatStatus msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "ethercat";
    msg.master_active = true;
    msg.link_up = true;
    msg.domain_wc_state = 2;
    msg.phase = statuses.size() == 1 ? statuses.front().phase : "multi";

    for (const auto& status : statuses) {
        msg.master_active = msg.master_active && status.master_active;
        msg.link_up = msg.link_up && status.link_up;
        msg.slaves_responding += status.slaves_responding;
        msg.al_states |= status.al_states;
        msg.domain_working_counter += status.domain_working_counter;
        msg.domain_wc_state = std::min(msg.domain_wc_state, status.domain_wc_state);
    }

    if (!msg.master_active) {
        msg.state_text = "EtherCAT master inactive";
    } else if (!msg.link_up) {
        msg.state_text = "EtherCAT link down";
    } else if (msg.slaves_responding == 0) {
        msg.state_text = "No responding EtherCAT slaves";
    } else if (statuses.size() == 1) {
        msg.state_text = statuses.front().state_text;
    } else {
        msg.state_text = "EtherCAT multi-master status";
    }

    ethercat_status_publisher_->publish(msg);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorManagerNode>());
    rclcpp::shutdown();
    return 0;
}
