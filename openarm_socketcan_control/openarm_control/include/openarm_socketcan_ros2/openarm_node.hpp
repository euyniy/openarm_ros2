#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <openarm_interfaces/msg/mit_command.hpp>
#include <openarm_interfaces/msg/mit_command_array.hpp>
#include <openarm_interfaces/msg/arm_command.hpp>
#include <openarm_interfaces/msg/gripper_command.hpp>
#include <openarm_interfaces/srv/query_param.hpp>

#include <openarm_control/openarm.hpp>
#include <damiao_motor/dm_motor_constants.hpp>
#include <damiao_motor/dm_motor_control.hpp>

#include <memory>
#include <vector>
#include <string>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>

namespace openarm_socketcan_ros2
{

class OpenArmNode : public rclcpp::Node
{
public:
    explicit OpenArmNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    virtual ~OpenArmNode();

private:
    // === Initialization ===
    void declare_parameters();
    void load_parameters();
    void init_openarm();
    void init_components();
    void setup_publishers();
    void setup_subscribers();
    void setup_services();
    void start_threads();

    // === Core control loops ===
    void socketcan_fd_monitor_loop();

    // === Joint state publishing ===
    void publish_joint_states();

    // === CAN socket monitoring ===
    void process_can_data_available();
    int get_can_socket_fd();

    // === Component service callbacks ===
    // Arm services
    void arm_enable_callback(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                            std::shared_ptr<std_srvs::srv::Empty::Response> response);
    void arm_disable_callback(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                             std::shared_ptr<std_srvs::srv::Empty::Response> response);
    void arm_set_zero_callback(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                              std::shared_ptr<std_srvs::srv::Empty::Response> response);
    void arm_query_params_callback(const std::shared_ptr<openarm_interfaces::srv::QueryParam::Request> request,
                                  std::shared_ptr<openarm_interfaces::srv::QueryParam::Response> response);

    // Gripper services
    void gripper_enable_callback(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                                std::shared_ptr<std_srvs::srv::Empty::Response> response);
    void gripper_disable_callback(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                                 std::shared_ptr<std_srvs::srv::Empty::Response> response);
    void gripper_set_zero_callback(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                                  std::shared_ptr<std_srvs::srv::Empty::Response> response);
    void gripper_query_params_callback(const std::shared_ptr<openarm_interfaces::srv::QueryParam::Request> request,
                                      std::shared_ptr<openarm_interfaces::srv::QueryParam::Response> response);

    // === Command callbacks ===
    void arm_command_callback(const openarm_interfaces::msg::ArmCommand::SharedPtr msg);
    void gripper_command_callback(const openarm_interfaces::msg::GripperCommand::SharedPtr msg);

    // === Utility methods ===
    damiao_motor::DM_Motor_Type string_to_motor_type(const std::string& type_str);
    std::vector<damiao_motor::MITParam> create_mit_params(const std::vector<double>& positions,
                                                         const std::vector<double>& velocities,
                                                         const std::vector<double>& efforts,
                                                         const std::vector<double>& kp_values,
                                                         const std::vector<double>& kd_values);

    // === Core configuration ===
    std::string arm_namespace_;
    std::string can_interface_;
    bool enable_canfd_;

    // === Motor configuration ===
    std::vector<std::string> arm_motor_types_;
    std::vector<uint32_t> arm_send_ids_;
    std::vector<uint32_t> arm_recv_ids_;
    std::vector<std::string> arm_joint_names_;

    // === Gripper configuration ===
    bool enable_gripper_;
    std::string gripper_motor_type_;
    uint32_t gripper_send_id_;
    uint32_t gripper_recv_id_;
    std::string gripper_joint_name_;

    // === Control parameters ===
    std::vector<double> arm_default_kp_;
    std::vector<double> arm_default_kd_;
    double gripper_default_kp_;
    double gripper_default_kd_;

    // === Core objects ===
    std::unique_ptr<OpenArm> openarm_;

    // === Publishers ===
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;

    // === Subscribers ===
    rclcpp::Subscription<openarm_interfaces::msg::ArmCommand>::SharedPtr arm_command_sub_;
    rclcpp::Subscription<openarm_interfaces::msg::GripperCommand>::SharedPtr gripper_command_sub_;

    // === Services ===
    // Arm services
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr arm_enable_srv_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr arm_disable_srv_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr arm_set_zero_srv_;
    rclcpp::Service<openarm_interfaces::srv::QueryParam>::SharedPtr arm_query_params_srv_;

    // Gripper services
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr gripper_enable_srv_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr gripper_disable_srv_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr gripper_set_zero_srv_;
    rclcpp::Service<openarm_interfaces::srv::QueryParam>::SharedPtr gripper_query_params_srv_;

    // === Thread management ===
    std::unique_ptr<std::thread> socketcan_thread_;
    std::atomic<bool> should_stop_threads_;
    std::mutex state_mutex_;  // Protect joint state publishing

    // === State tracking ===
    sensor_msgs::msg::JointState current_joint_state_;
    std::chrono::steady_clock::time_point last_command_time_;
};

} // namespace openarm_socketcan_ros2
