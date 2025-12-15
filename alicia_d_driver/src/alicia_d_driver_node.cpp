#include "alicia_d_driver/alicia_d_driver_node.hpp"
#include <cmath>


AliciaDDriverNode::AliciaDDriverNode(const rclcpp::NodeOptions& options)
    : Node("alicia_d_driver_node", options)
{
    declare_parameters();
    setup_ros_communications();

    // Attempt initial connection
    if (communicator_->connect()) {
        
        data_parser_control_->start_parsing_thread();

        // Query all information types 
        data_parser_control_->acquire_info("version", true, 3.0, 0.2);
        data_parser_control_->acquire_info("temperature", true, 2.0, 0.2);
        data_parser_control_->acquire_info("velocity", true, 2.0, 0.2);
        data_parser_control_->acquire_info("self_check", true, 2.0, 0.2);
        
        // Print all available information
        data_parser_control_->print_information();

    } else {
        RCLCPP_ERROR(this->get_logger(), "Initial connection failed. Starting reconnect timer.");
        reconnect_timer_ = this->create_wall_timer(
            std::chrono::seconds(5),
            [this]() {
                if (!communicator_->is_connected()) {
                    RCLCPP_INFO(this->get_logger(), "Attempting to reconnect...");
                    if (communicator_->connect()) {
                        RCLCPP_INFO(this->get_logger(), "Reconnect successful! Starting parsing thread.");
                        // Start parsing thread after reconnect
                        data_parser_control_->start_parsing_thread();
                        reconnect_timer_->cancel();
                    }
                } else {
                    reconnect_timer_->cancel();
                }
            }
        );
    }
}

AliciaDDriverNode::~AliciaDDriverNode()
{
    if (data_parser_control_) {
        data_parser_control_->stop_parsing_thread();
    }
    if (communicator_) {
        communicator_->disconnect();
    }
}


void AliciaDDriverNode::declare_parameters()
{
    if (!this->has_parameter("port")) {
        this->declare_parameter<std::string>("port", "");
    }
    if (!this->has_parameter("default_speed_deg_s")) {
        this->declare_parameter<double>("default_speed_deg_s", 20.0);
    }

    std::string port = this->get_parameter("port").as_string();
    default_speed_deg_s_ = this->get_parameter("default_speed_deg_s").as_double();
    
    // Use hardcoded defaults for other parameters
    debug_mode_ = false;

    communicator_ = std::make_unique<SerialCommunicator>(port, debug_mode_);
    // Gripper type is not configurable in driver node (use hardware interface for that)
    data_parser_control_ = std::make_unique<AliciaDDataParserControl>(
        communicator_.get(), this->get_logger(), debug_mode_, "50mm");  // Default to 50mm
    RCLCPP_INFO(this->get_logger(), "Configured port: %s, baud: %u (fixed), debug: %s, default_speed: %.1f deg/s",
                port.c_str(), 1000000, debug_mode_ ? "true" : "false", default_speed_deg_s_);
}

void AliciaDDriverNode::setup_ros_communications()
{
    // Publishers
    joint_state_pub_std_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

    joint_command_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_commands", 10, std::bind(&AliciaDDriverNode::joint_command_callback, this, std::placeholders::_1));
        
    zero_calib_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/zero_calibrate", 10, std::bind(&AliciaDDriverNode::zero_calibrate_callback, this, std::placeholders::_1));
    demo_mode_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/demonstration", 10, std::bind(&AliciaDDriverNode::demonstration_mode_callback, this, std::placeholders::_1));

    
    // Heartbeat publisher to keep /joint_states fresh (reads from parser and publishes)
    heartbeat_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),  // 100 Hz publishing rate
        std::bind(&AliciaDDriverNode::heartbeat_publish_callback, this));
    
    // Periodically request joint data from robot (non-blocking)
    joint_request_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),  // Request joint data at 100 Hz
        [this]() {
            if (communicator_ && communicator_->is_connected()) {
                // Request joint data (non-blocking, parsing happens in background thread)
                data_parser_control_->acquire_info("joint", false);
            }
        });
    
}


void AliciaDDriverNode::joint_command_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    if (!communicator_ || !communicator_->is_connected()) {
        return;
    }
    
    std::map<std::string, double> joint_pos_map;
    std::map<std::string, double> joint_vel_map;
    
    // Extract positions
    for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
        joint_pos_map[msg->name[i]] = msg->position[i];
    }
    
    // Extract velocities (if provided)
    for (size_t i = 0; i < msg->name.size() && i < msg->velocity.size(); ++i) {
        joint_vel_map[msg->name[i]] = msg->velocity[i];
    }
    
    std::vector<std::string> hardware_joint_names = {"Joint1","Joint2","Joint3","Joint4","Joint5","Joint6"};
    std::vector<double> joint_angles;
    
    // Extract joint positions
    for (const auto& joint_name : hardware_joint_names) {
        auto it = joint_pos_map.find(joint_name);
        joint_angles.push_back(it != joint_pos_map.end() ? it->second : 0.0);
    }
    
    // Extract single speed value from velocities 
    double speed_deg_s = default_speed_deg_s_;  // Default speed in deg/s
    bool has_velocity = false;
    double max_abs_vel = 0.0;
    for (const auto& joint_name : hardware_joint_names) {
        auto vel_it = joint_vel_map.find(joint_name);
        if (vel_it != joint_vel_map.end()) {
            double abs_vel = std::abs(vel_it->second) * 180.0 / M_PI;  // Convert rad/s to deg/s
            if (abs_vel > 1e-6) {
                has_velocity = true;
                max_abs_vel = std::max(max_abs_vel, abs_vel);
            }
        }
    }
    if (has_velocity) {
        speed_deg_s = max_abs_vel;  // Use maximum velocity as the common speed for all joints
    }
    
    // Extract gripper value directly (0-1000)
    double gripper_value = -1.0;  // -1.0 means use current 
    auto it_grip = joint_pos_map.find("Gripper");
    if (it_grip != joint_pos_map.end()) {
        gripper_value = std::max(0.0, std::min(1000.0, it_grip->second));
    }
    
    data_parser_control_->set_joint_and_gripper(joint_angles, gripper_value, speed_deg_s);
}




void AliciaDDriverNode::zero_calibrate_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (msg->data) {
        RCLCPP_INFO(this->get_logger(), "Received Zero Calibration command.");
        data_parser_control_->zero_calibration();
        std::this_thread::sleep_for(std::chrono::microseconds(4));
        data_parser_control_->torque_control("on");
    }
}

void AliciaDDriverNode::demonstration_mode_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (msg->data) {
        RCLCPP_INFO(this->get_logger(), "Enabling Demonstration Mode (Zero Torque).");
        data_parser_control_->torque_control("off");
    } else {
        RCLCPP_INFO(this->get_logger(), "Disabling Demonstration Mode (Full Torque).");
        data_parser_control_->torque_control("on");
    }
}



int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<AliciaDDriverNode>(options);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

void AliciaDDriverNode::heartbeat_publish_callback()
{
    if (!communicator_ || !communicator_->is_connected()) {
        return;
    }
    
    // Read joint state from parser (real-time data)
    auto joint_state = data_parser_control_->get_joint_state();
    auto velocity_data = data_parser_control_->get_velocity_data();
    
    auto now = this->get_clock()->now();
    sensor_msgs::msg::JointState js;
    js.header.stamp = now;
    js.name = joint_names_;
    
    if (joint_state.has_value()) {
        // Publish joint positions (6 joints + gripper)
        js.position = joint_state->angles;
        js.position.push_back(data_parser_control_->gripper_value_to_position(joint_state->gripper));
        
        // Publish velocities if available
        if (velocity_data.has_value() && velocity_data->velocities.size() >= 6) {
            js.velocity = velocity_data->velocities;
            // Gripper velocity is typically not available, add 0.0
            js.velocity.push_back(0.0);
        }
    } else {
        // No data available yet, publish zeros
        js.position = std::vector<double>(7, 0.0);
    }
    
    joint_state_pub_std_->publish(js);
}
