#ifndef ALICIA_D_DRIVER_NODE_HPP
#define ALICIA_D_DRIVER_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "serial_communicator.hpp"
#include "alicia_d_data_parser_control.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include <memory>
#include <vector>

class AliciaDDriverNode : public rclcpp::Node
{
public:
    explicit AliciaDDriverNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~AliciaDDriverNode();

private:
    // Initialization
    void declare_parameters();
    void setup_ros_communications();

    // Callbacks for incoming commands
    void joint_command_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void zero_calibrate_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void demonstration_mode_callback(const std_msgs::msg::Bool::SharedPtr msg);

    // Main processing loop
    void heartbeat_publish_callback();

    // Member Variables
    std::unique_ptr<SerialCommunicator> communicator_;
    std::unique_ptr<AliciaDDataParserControl> data_parser_control_;
    rclcpp::TimerBase::SharedPtr reconnect_timer_;
    rclcpp::TimerBase::SharedPtr heartbeat_timer_;
    rclcpp::TimerBase::SharedPtr joint_request_timer_;

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_std_;

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_command_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr zero_calib_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr demo_mode_sub_;

    // Configuration
    bool debug_mode_ = false;
    double default_speed_deg_s_ = 20.0;

    // State for heartbeat
    std::vector<std::string> joint_names_ = {"Joint1","Joint2","Joint3","Joint4","Joint5","Joint6","Gripper"};
};

#endif // ALICIA_D_DRIVER_NODE_HPP

