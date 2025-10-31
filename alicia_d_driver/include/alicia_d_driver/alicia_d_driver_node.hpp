#ifndef ALICIA_D_DRIVER_NODE_HPP
#define ALICIA_D_DRIVER_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "serial_communicator.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include <memory>
#include <vector>

class AliciaDDriverNode : public rclcpp::Node
{
public:
    AliciaDDriverNode();
    explicit AliciaDDriverNode(const rclcpp::NodeOptions& options);
    ~AliciaDDriverNode();

private:
    // Initialization
    void declare_parameters();
    void setup_ros_communications();

    // Callbacks for incoming commands
    // void joint_command_callback(const alicia_d_driver::msg::ArmJointState::SharedPtr msg);
    void joint_command_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void zero_calibrate_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void demonstration_mode_callback(const std_msgs::msg::Bool::SharedPtr msg);

    // Main processing loop
    void process_serial_data();
    void heartbeat_publish_callback();
    void send_command_timer_callback();

    // Firmware handling
    void send_firmware_query();
    void parse_version_frame(const std::vector<uint8_t>& data_payload);

    // Data Conversion & Framing (from arm_control_node.py)
    uint16_t rad_to_hardware_value(double angle_rad);
    uint16_t rad_to_hardware_value_grip(double angle_rad);
    uint8_t calculate_checksum(const std::vector<uint8_t>& frame_data); // Add this

    // Data Parsing (from joint_state_publisher_node.py)
    double hardware_value_to_rad(uint16_t hw_value);
    double hardware_value_to_rad_grip(uint16_t hw_value);
    void parse_servo_states_frame(const std::vector<uint8_t>& payload);
    void parse_gripper_state_frame(const std::vector<uint8_t>& payload); // Add this
    void parse_error_frame(const std::vector<uint8_t>& payload); // Add this

    std::vector<uint8_t> generate_simple_frame(uint8_t command, uint8_t data, bool use_checksum);
    // Member Variables
    std::unique_ptr<SerialCommunicator> communicator_;
    rclcpp::TimerBase::SharedPtr processing_timer_;
    rclcpp::TimerBase::SharedPtr reconnect_timer_;
    rclcpp::TimerBase::SharedPtr heartbeat_timer_;
    rclcpp::TimerBase::SharedPtr command_timer_;

    // Publishers
    // rclcpp::Publisher<alicia_d_driver::msg::ArmJointState>::SharedPtr joint_state_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr array_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_std_;
    // Subscribers
    // rclcpp::Subscription<alicia_d_driver::msg::ArmJointState>::SharedPtr joint_command_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_command_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr zero_calib_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr demo_mode_sub_;

    // State
    std::vector<double> joint_to_servo_map_direction_;
    std::vector<int> joint_to_servo_map_index_;
    std::vector<int> servo_to_joint_map_index_;
    std::vector<double> servo_to_joint_map_direction_;
    int servo_count_;
    bool debug_mode_;
    double rate_limit_sec_;
    rclcpp::Time last_process_time_;

    // Command handling (decoupled sender)
    std::vector<double> latest_joint_angles_;
    double latest_gripper_value_ = 0.0; // 0..100 percent open
    std::mutex latest_cmd_mutex_;
    bool has_latest_command_ = false;

    // Versioning / firmware
    bool firmware_version_detected_ = false;
    bool firmware_new_ = false; // true for v6+
    std::string firmware_version_;
    std::string gripper_type_ = "50mm";
    double default_speed_rad_s_ = 0.349;

    // ROS1-like parameters
    double command_rate_hz_ = 200.0;
    bool use_trajectory_smoothing_ = false;
    double max_joint_velocity_rad_s_ = 5.0;
    double max_gripper_velocity_units_s_ = 100.0; // 0..100 units per second
    double max_joint_accel_rad_s2_ = 20.0;
    double max_gripper_accel_units_s2_ = 100.0;

    // State for heartbeat
    std::vector<std::string> joint_names_ = {"Joint1","Joint2","Joint3","Joint4","Joint5","Joint6","right_finger"};
    std::vector<double> current_joint_positions_{0.0,0.0,0.0,0.0,0.0,0.0};
    double current_gripper_position_m_ = 0.0; // meters for prismatic
    std::mutex data_mutex_;
};

#endif // ALICIA_D_DRIVER_NODE_HPP

