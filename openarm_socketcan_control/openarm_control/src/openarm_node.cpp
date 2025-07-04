#include "openarm_socketcan_ros2/openarm_node.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include <sys/select.h>
#include <unistd.h>
#include <errno.h>
#include <cstring>
#include <mutex>

/*
 *
 * This node uses a hybrid threading approach to eliminate race conditions:
 *
 * 1. SINGLE-THREADED ROS EXECUTOR:
 *    - All ROS callbacks (services, subscribers) run sequentially in main thread
 *    - All CAN socket WRITES are serialized → no write race conditions
 *    - Services: enable/disable/set_zero/query_params
 *    - Subscribers: arm_command/gripper_command
 *
 * 2. DEDICATED SOCKETCAN READ THREAD:
 *    - Event-driven CAN socket monitoring using select()
 *    - Only thread that calls recv_all() → reads CAN data and updates motor state
 *    - Publishes joint states immediately when fresh data arrives
 *    - Real-time performance without polling overhead
 *
 * THREAD INTERACTIONS:
 * - Write thread (ROS executor): Commands → CAN socket
 * - Read thread (SocketCAN): CAN socket → Motor state updates → Joint state publishing
 * - Motor state access is protected by state_mutex_ during publishing
 */

namespace openarm_socketcan_ros2
{

OpenArmNode::OpenArmNode(const rclcpp::NodeOptions & options)
: Node("openarm_node", options), should_stop_threads_(false)
{
    RCLCPP_INFO(this->get_logger(), "Initializing OpenArm Node");

    declare_parameters();
    load_parameters();
    init_openarm();
    init_components();
    setup_publishers();
    setup_subscribers();
    setup_services();
    start_threads();

    RCLCPP_INFO(this->get_logger(), "OpenArm Node initialized successfully");
}

OpenArmNode::~OpenArmNode()
{
    RCLCPP_INFO(this->get_logger(), "Shutting down OpenArm Node");

    should_stop_threads_ = true;

    if (socketcan_thread_ && socketcan_thread_->joinable()) {
        socketcan_thread_->join();
    }

    if (openarm_) {
        openarm_->disable_all();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        openarm_->recv_all();
    }
}

void OpenArmNode::declare_parameters()
{
    // Core configuration
    std::string default_namespace = std::string(this->get_namespace());
    if (default_namespace.empty() || default_namespace == "/") {
        default_namespace = "openarm";
    }
    this->declare_parameter("arm_namespace", default_namespace);
    this->declare_parameter("can_interface", "can0");
    this->declare_parameter("enable_canfd", true);

    // Motor configuration
    this->declare_parameter("arm_motor_types", std::vector<std::string>{"DM4310", "DM4310"});
    this->declare_parameter("arm_send_ids", std::vector<int64_t>{0x01, 0x02});
    this->declare_parameter("arm_recv_ids", std::vector<int64_t>{0x11, 0x12});
    this->declare_parameter("arm_joint_names", std::vector<std::string>{"arm_joint_1", "arm_joint_2"});

    // Gripper configuration
    this->declare_parameter("enable_gripper", true);
    this->declare_parameter("gripper_motor_type", "DM4310");
    this->declare_parameter("gripper_send_id", 0x07);
    this->declare_parameter("gripper_recv_id", 0x17);
    this->declare_parameter("gripper_joint_name", "gripper_joint");

    // Control parameters
    this->declare_parameter("arm_default_kp", std::vector<double>{2.0, 2.0});
    this->declare_parameter("arm_default_kd", std::vector<double>{1.0, 1.0});
    this->declare_parameter("gripper_default_kp", 50.0);
    this->declare_parameter("gripper_default_kd", 1.0);
}

void OpenArmNode::load_parameters()
{
    // Core configuration
    arm_namespace_ = this->get_parameter("arm_namespace").as_string();
    can_interface_ = this->get_parameter("can_interface").as_string();
    enable_canfd_ = this->get_parameter("enable_canfd").as_bool();

    // Motor configuration
    arm_motor_types_ = this->get_parameter("arm_motor_types").as_string_array();
    auto arm_send_ids_int64 = this->get_parameter("arm_send_ids").as_integer_array();
    auto arm_recv_ids_int64 = this->get_parameter("arm_recv_ids").as_integer_array();

    arm_send_ids_.clear();
    arm_recv_ids_.clear();
    for (auto id : arm_send_ids_int64) {
        arm_send_ids_.push_back(static_cast<uint32_t>(id));
    }
    for (auto id : arm_recv_ids_int64) {
        arm_recv_ids_.push_back(static_cast<uint32_t>(id));
    }

    arm_joint_names_ = this->get_parameter("arm_joint_names").as_string_array();

    // Gripper configuration
    enable_gripper_ = this->get_parameter("enable_gripper").as_bool();
    gripper_motor_type_ = this->get_parameter("gripper_motor_type").as_string();
    gripper_send_id_ = static_cast<uint32_t>(this->get_parameter("gripper_send_id").as_int());
    gripper_recv_id_ = static_cast<uint32_t>(this->get_parameter("gripper_recv_id").as_int());
    gripper_joint_name_ = this->get_parameter("gripper_joint_name").as_string();

    // Control parameters
    arm_default_kp_ = this->get_parameter("arm_default_kp").as_double_array();
    arm_default_kd_ = this->get_parameter("arm_default_kd").as_double_array();
    gripper_default_kp_ = this->get_parameter("gripper_default_kp").as_double();
    gripper_default_kd_ = this->get_parameter("gripper_default_kd").as_double();

    RCLCPP_INFO(this->get_logger(), "Parameters loaded: CAN interface: %s, Arm joints: %zu, Gripper enabled: %s",
                can_interface_.c_str(), arm_joint_names_.size(), enable_gripper_ ? "true" : "false");
}

void OpenArmNode::init_openarm()
{
    try {
        openarm_ = std::make_unique<OpenArm>(can_interface_, enable_canfd_);
        RCLCPP_INFO(this->get_logger(), "OpenArm initialized with interface: %s", can_interface_.c_str());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize OpenArm: %s", e.what());
        throw;
    }
}

void OpenArmNode::init_components()
{
    // Initialize arm motors
    if (!arm_motor_types_.empty()) {
        std::vector<damiao_motor::DM_Motor_Type> motor_types;
        for (const auto& type_str : arm_motor_types_) {
            motor_types.push_back(string_to_motor_type(type_str));
        }

        openarm_->init_arm_motors(motor_types, arm_send_ids_, arm_recv_ids_);
        RCLCPP_INFO(this->get_logger(), "Initialized %zu arm motors", motor_types.size());
    }

    // Initialize gripper motor
    if (enable_gripper_) {
        auto gripper_type = string_to_motor_type(gripper_motor_type_);
        openarm_->init_gripper_motor(gripper_type, gripper_send_id_, gripper_recv_id_);
        RCLCPP_INFO(this->get_logger(), "Initialized gripper motor");
    }

    // Set callback mode to STATE for real-time control
    openarm_->set_callback_mode_all(damiao_motor::CallbackMode::STATE);

    // Initialize joint state message
    current_joint_state_.header.frame_id = arm_namespace_ + "_base_link";
    current_joint_state_.name = arm_joint_names_;
    if (enable_gripper_) {
        current_joint_state_.name.push_back(gripper_joint_name_);
    }

    size_t total_joints = current_joint_state_.name.size();
    current_joint_state_.position.resize(total_joints, 0.0);
    current_joint_state_.velocity.resize(total_joints, 0.0);
    current_joint_state_.effort.resize(total_joints, 0.0);
}

void OpenArmNode::setup_publishers()
{
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
        arm_namespace_ + "/joint_states", 10);
}

void OpenArmNode::setup_subscribers()
{
    // Use default callback group (single-threaded executor)
    arm_command_sub_ = this->create_subscription<openarm_interfaces::msg::ArmCommand>(
        arm_namespace_ + "/arm_command", 10,
        std::bind(&OpenArmNode::arm_command_callback, this, std::placeholders::_1));

    if (enable_gripper_) {
        gripper_command_sub_ = this->create_subscription<openarm_interfaces::msg::GripperCommand>(
            arm_namespace_ + "/gripper_command", 10,
            std::bind(&OpenArmNode::gripper_command_callback, this, std::placeholders::_1));
    }
}

void OpenArmNode::setup_services()
{
    // Use default callback group (single-threaded executor)
    // Arm services
    arm_enable_srv_ = this->create_service<std_srvs::srv::Empty>(
        arm_namespace_ + "/arm/enable",
        std::bind(&OpenArmNode::arm_enable_callback, this, std::placeholders::_1, std::placeholders::_2));

    arm_disable_srv_ = this->create_service<std_srvs::srv::Empty>(
        arm_namespace_ + "/arm/disable",
        std::bind(&OpenArmNode::arm_disable_callback, this, std::placeholders::_1, std::placeholders::_2));

    arm_set_zero_srv_ = this->create_service<std_srvs::srv::Empty>(
        arm_namespace_ + "/arm/set_zero",
        std::bind(&OpenArmNode::arm_set_zero_callback, this, std::placeholders::_1, std::placeholders::_2));

    arm_query_params_srv_ = this->create_service<openarm_interfaces::srv::QueryParam>(
        arm_namespace_ + "/arm/query_params",
        std::bind(&OpenArmNode::arm_query_params_callback, this, std::placeholders::_1, std::placeholders::_2));

    // Gripper services
    if (enable_gripper_) {
        gripper_enable_srv_ = this->create_service<std_srvs::srv::Empty>(
            arm_namespace_ + "/gripper/enable",
            std::bind(&OpenArmNode::gripper_enable_callback, this, std::placeholders::_1, std::placeholders::_2));

        gripper_disable_srv_ = this->create_service<std_srvs::srv::Empty>(
            arm_namespace_ + "/gripper/disable",
            std::bind(&OpenArmNode::gripper_disable_callback, this, std::placeholders::_1, std::placeholders::_2));

        gripper_set_zero_srv_ = this->create_service<std_srvs::srv::Empty>(
            arm_namespace_ + "/gripper/set_zero",
            std::bind(&OpenArmNode::gripper_set_zero_callback, this, std::placeholders::_1, std::placeholders::_2));

        gripper_query_params_srv_ = this->create_service<openarm_interfaces::srv::QueryParam>(
            arm_namespace_ + "/gripper/query_params",
            std::bind(&OpenArmNode::gripper_query_params_callback, this, std::placeholders::_1, std::placeholders::_2));
    }
}

void OpenArmNode::start_threads()
{   // TODO can we assume only one for now? not very extensible
    // SocketCAN file descriptor monitoring thread - publishes joint states when data is received
    socketcan_thread_ = std::make_unique<std::thread>(
        std::bind(&OpenArmNode::socketcan_fd_monitor_loop, this));
}

void OpenArmNode::socketcan_fd_monitor_loop()
{
    RCLCPP_INFO(this->get_logger(), "SocketCAN file descriptor monitoring thread started");

    /* Event-driven CAN processing using select():
     * - Eliminates unnecessary polling and CPU usage
     * - Provides immediate response to incoming CAN messages
     * - More efficient than timer-based approaches
     * - Scales better with multiple CAN interfaces
     * - Publishes joint states immediately when new data arrives
     */
    int can_fd = get_can_socket_fd();
    if (can_fd < 0) {
        RCLCPP_ERROR(this->get_logger(), "Invalid CAN socket file descriptor");
        return;
    }

    fd_set read_fds;
    struct timeval timeout;

    while (!should_stop_threads_) {
        try {
            FD_ZERO(&read_fds);
            FD_SET(can_fd, &read_fds);

            // Set timeout to 100ms so we can check should_stop_threads_ regularly
            timeout.tv_sec = 0;
            timeout.tv_usec = 100000; // 100ms
            // TODO double check this. or add_waitable or epoll or something else
            int select_result = select(can_fd + 1, &read_fds, nullptr, nullptr, &timeout);

            if (select_result > 0 && FD_ISSET(can_fd, &read_fds)) {
                // Data is available on CAN socket
                // TODO this calls recv_all make sure that thread is managed properly
                process_can_data_available();
            } else if (select_result < 0) {
                RCLCPP_WARN(this->get_logger(), "select() error on CAN socket: %s", strerror(errno));
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            // select_result == 0 means timeout, which is normal - just continue loop

        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Error in SocketCAN file descriptor monitoring loop: %s", e.what());
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    RCLCPP_INFO(this->get_logger(), "SocketCAN file descriptor monitoring thread stopped");
}

void OpenArmNode::process_can_data_available()
{
    if (openarm_) {
        // Process incoming CAN data - this is the only thread that calls recv_all()
        openarm_->recv_all();

        // Publish joint states immediately with fresh data
        publish_joint_states();
    }
}

int OpenArmNode::get_can_socket_fd()
{
    if (!openarm_) {
        return -1;
    }

    try {
        // Get the socket FD from the OpenArm's CAN device collection
        return openarm_->get_master_can_device_collection().get_socket_fd();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get CAN socket FD: %s", e.what());
        return -1;
    }
}

void OpenArmNode::publish_joint_states()
{
    std::lock_guard<std::mutex> lock(state_mutex_);

    current_joint_state_.header.stamp = this->now();

    // Update arm joint states
    if (!arm_joint_names_.empty()) {
        const auto& arm_motors = openarm_->get_arm().get_motors();
        for (size_t i = 0; i < arm_motors.size() && i < arm_joint_names_.size(); ++i) {
            current_joint_state_.position[i] = arm_motors[i].get_position();
            current_joint_state_.velocity[i] = arm_motors[i].get_velocity();
            current_joint_state_.effort[i] = arm_motors[i].get_torque();
        }
    }

    // Update gripper joint state
    if (enable_gripper_ && openarm_->get_gripper().has_motor()) {
        size_t gripper_idx = arm_joint_names_.size();
        const auto* gripper_motor = openarm_->get_gripper().get_motor();
        if (gripper_motor) {
            current_joint_state_.position[gripper_idx] = gripper_motor->get_position();
            current_joint_state_.velocity[gripper_idx] = gripper_motor->get_velocity();
            current_joint_state_.effort[gripper_idx] = gripper_motor->get_torque();
        }
    }

    joint_state_pub_->publish(current_joint_state_);
}

// === Arm Service Callbacks ===

void OpenArmNode::arm_enable_callback(
    const std::shared_ptr<std_srvs::srv::Empty::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Empty::Response> /*response*/)
{
    try {
        openarm_->get_arm().enable_all();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        RCLCPP_INFO(this->get_logger(), "Arm motors enabled");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to enable arm motors: %s", e.what());
    }
}

void OpenArmNode::arm_disable_callback(
    const std::shared_ptr<std_srvs::srv::Empty::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Empty::Response> /*response*/)
{
    try {
        openarm_->get_arm().disable_all();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        RCLCPP_INFO(this->get_logger(), "Arm motors disabled");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to disable arm motors: %s", e.what());
    }
}

void OpenArmNode::arm_set_zero_callback(
    const std::shared_ptr<std_srvs::srv::Empty::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Empty::Response> /*response*/)
{
    try {
        openarm_->get_arm().set_zero_all();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        RCLCPP_INFO(this->get_logger(), "Arm motor positions set to zero");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set arm motor zeros: %s", e.what());
    }
}

void OpenArmNode::arm_query_params_callback(
    const std::shared_ptr<openarm_interfaces::srv::QueryParam::Request> request,
    std::shared_ptr<openarm_interfaces::srv::QueryParam::Response> response)
{
    try {
        // Validate motor_id parameter
        if (request->motor_id < 0 || static_cast<size_t>(request->motor_id) >= arm_joint_names_.size()) {
            RCLCPP_ERROR(this->get_logger(), "Invalid arm motor_id: %d (valid range: 0-%zu)",
                        request->motor_id, arm_joint_names_.size() - 1);
            response->value = -1.0;
            return;
        }

        // Set callback mode to PARAM for the arm
        openarm_->get_arm().set_callback_mode_all(damiao_motor::CallbackMode::PARAM);

        // Send query param command to specific motor
        openarm_->get_arm().query_param_one(request->motor_id, request->rid);

        // Wait for response to be processed by the socketcan thread
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // Get the parameter value from the specific motor
        const auto& arm_motors = openarm_->get_arm().get_motors();
        double param_value = arm_motors[request->motor_id].get_param(request->rid);

        // Switch back to STATE mode
        openarm_->get_arm().set_callback_mode_all(damiao_motor::CallbackMode::STATE);

        response->value = param_value;

        RCLCPP_INFO(this->get_logger(), "Arm motor %d parameter RID %d value: %f",
                   request->motor_id, request->rid, param_value);

    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to query arm motor parameter: %s", e.what());
        // Ensure we switch back to STATE mode even on error
        try {
            openarm_->get_arm().set_callback_mode_all(damiao_motor::CallbackMode::STATE);
        } catch (...) {}
        response->value = -1.0;
    }
}

// === Gripper Service Callbacks ===

void OpenArmNode::gripper_enable_callback(
    const std::shared_ptr<std_srvs::srv::Empty::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Empty::Response> /*response*/)
{
    try {
        openarm_->get_gripper().enable_all();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        RCLCPP_INFO(this->get_logger(), "Gripper motor enabled");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to enable gripper motor: %s", e.what());
    }
}

void OpenArmNode::gripper_disable_callback(
    const std::shared_ptr<std_srvs::srv::Empty::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Empty::Response> /*response*/)
{
    try {
        openarm_->get_gripper().disable_all();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        RCLCPP_INFO(this->get_logger(), "Gripper motor disabled");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to disable gripper motor: %s", e.what());
    }
}

void OpenArmNode::gripper_set_zero_callback(
    const std::shared_ptr<std_srvs::srv::Empty::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Empty::Response> /*response*/)
{
    try {
        openarm_->get_gripper().set_zero_all();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        RCLCPP_INFO(this->get_logger(), "Gripper motor position set to zero");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set gripper motor zero: %s", e.what());
    }
}

void OpenArmNode::gripper_query_params_callback(
    const std::shared_ptr<openarm_interfaces::srv::QueryParam::Request> request,
    std::shared_ptr<openarm_interfaces::srv::QueryParam::Response> response)
{
    try {
        if (!enable_gripper_) {
            RCLCPP_ERROR(this->get_logger(), "Gripper is not enabled");
            response->value = -1.0;
            return;
        }

        // Validate motor_id parameter (gripper typically has only one motor, so motor_id should be 0)
        if (request->motor_id != 0) {
            RCLCPP_ERROR(this->get_logger(), "Invalid gripper motor_id: %d (gripper only has motor_id 0)",
                        request->motor_id);
            response->value = -1.0;
            return;
        }

        // Set callback mode to PARAM for the gripper
        openarm_->get_gripper().set_callback_mode_all(damiao_motor::CallbackMode::PARAM);

        // Send query param command to specific motor
        openarm_->get_gripper().query_param_one(request->motor_id, request->rid);

        // Wait for response to be processed by the socketcan thread
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // Get the parameter value from the gripper motor
        const auto& gripper_motors = openarm_->get_gripper().get_motors();
        double param_value = gripper_motors[request->motor_id].get_param(request->rid);

        // Switch back to STATE mode
        openarm_->get_gripper().set_callback_mode_all(damiao_motor::CallbackMode::STATE);

        response->value = param_value;

        RCLCPP_INFO(this->get_logger(), "Gripper motor %d parameter RID %d value: %f",
                   request->motor_id, request->rid, param_value);

    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to query gripper motor parameter: %s", e.what());
        // Ensure we switch back to STATE mode even on error
        try {
            openarm_->get_gripper().set_callback_mode_all(damiao_motor::CallbackMode::STATE);
        } catch (...) {}
        response->value = -1.0;
    }
}

// === Command Callbacks ===

void OpenArmNode::arm_command_callback(const openarm_interfaces::msg::ArmCommand::SharedPtr msg)
{
    try {
        if (msg->positions.size() != arm_joint_names_.size()) {
            RCLCPP_WARN(this->get_logger(), "Arm command position array size mismatch");
            return;
        }

        // Use provided kp/kd or defaults
        std::vector<double> kp_values = msg->kp.empty() ? arm_default_kp_ : msg->kp;
        std::vector<double> kd_values = msg->kd.empty() ? arm_default_kd_ : msg->kd;

        auto mit_params = create_mit_params(msg->positions, msg->velocities, msg->efforts,
                                          kp_values, kd_values);

        openarm_->get_arm().mit_control_all(mit_params);
        last_command_time_ = std::chrono::steady_clock::now();

    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error processing arm command: %s", e.what());
    }
}

void OpenArmNode::gripper_command_callback(const openarm_interfaces::msg::GripperCommand::SharedPtr msg)
{
    try {
        if (!enable_gripper_) return;

        double kp = msg->kp > 0 ? msg->kp : gripper_default_kp_;
        double kd = msg->kd > 0 ? msg->kd : gripper_default_kd_;

        damiao_motor::MITParam mit_param;
        mit_param.kp = kp;
        mit_param.kd = kd;
        mit_param.q = msg->position;
        mit_param.dq = msg->velocity;
        mit_param.tau = msg->effort;

        openarm_->get_gripper().mit_control_all({mit_param});
        last_command_time_ = std::chrono::steady_clock::now();

    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error processing gripper command: %s", e.what());
    }
}

// === Utility Methods ===

damiao_motor::DM_Motor_Type OpenArmNode::string_to_motor_type(const std::string& type_str)
{
    if (type_str == "DM3507") return damiao_motor::DM_Motor_Type::DM3507;
    if (type_str == "DM4310") return damiao_motor::DM_Motor_Type::DM4310;
    if (type_str == "DM4310_48V") return damiao_motor::DM_Motor_Type::DM4310_48V;
    if (type_str == "DM4340") return damiao_motor::DM_Motor_Type::DM4340;
    if (type_str == "DM4340_48V") return damiao_motor::DM_Motor_Type::DM4340_48V;
    if (type_str == "DM6006") return damiao_motor::DM_Motor_Type::DM6006;
    if (type_str == "DM8006") return damiao_motor::DM_Motor_Type::DM8006;
    if (type_str == "DM8009") return damiao_motor::DM_Motor_Type::DM8009;
    if (type_str == "DM10010L") return damiao_motor::DM_Motor_Type::DM10010L;
    if (type_str == "DM10010") return damiao_motor::DM_Motor_Type::DM10010;
    if (type_str == "DMH3510") return damiao_motor::DM_Motor_Type::DMH3510;
    if (type_str == "DMH6215") return damiao_motor::DM_Motor_Type::DMH6215;
    if (type_str == "DMG6220") return damiao_motor::DM_Motor_Type::DMG6220;

    RCLCPP_WARN(this->get_logger(), "Unknown motor type: %s, defaulting to DM4310", type_str.c_str());
    return damiao_motor::DM_Motor_Type::DM4310;
}

std::vector<damiao_motor::MITParam> OpenArmNode::create_mit_params(
    const std::vector<double>& positions,
    const std::vector<double>& velocities,
    const std::vector<double>& efforts,
    const std::vector<double>& kp_values,
    const std::vector<double>& kd_values)
{
    std::vector<damiao_motor::MITParam> mit_params;

    for (size_t i = 0; i < positions.size(); ++i) {
        damiao_motor::MITParam param;
        param.kp = (i < kp_values.size()) ? kp_values[i] : arm_default_kp_[0];
        param.kd = (i < kd_values.size()) ? kd_values[i] : arm_default_kd_[0];
        param.q = positions[i];
        param.dq = (i < velocities.size()) ? velocities[i] : 0.0;
        param.tau = (i < efforts.size()) ? efforts[i] : 0.0;
        mit_params.push_back(param);
    }

    return mit_params;
}

} // namespace openarm_socketcan_ros2

RCLCPP_COMPONENTS_REGISTER_NODE(openarm_socketcan_ros2::OpenArmNode)
