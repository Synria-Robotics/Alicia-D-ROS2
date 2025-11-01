#ifndef ALICIA_D_DRIVER__ALICIA_D_HARDWARE_INTERFACE_HPP_
#define ALICIA_D_DRIVER__ALICIA_D_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <mutex>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "serial_communicator.hpp"

using hardware_interface::return_type;

namespace alicia_d_driver
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class HARDWARE_INTERFACE_PUBLIC AliciaDHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(AliciaDHardwareInterface)

  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Serial communication
  std::unique_ptr<SerialCommunicator> communicator_;
  std::string port_;
  int baud_rate_;
  bool debug_mode_;
  
  // Robot state
  std::vector<double> hw_positions_state_;
  std::vector<double> hw_positions_command_;
  
  // Joint and servo mapping
  std::vector<int> joint_to_servo_map_index_;
  std::vector<double> joint_to_servo_map_direction_;
  std::vector<int> servo_to_joint_map_index_;
  std::vector<double> servo_to_joint_map_direction_;
  int servo_count_;
  
  // Gripper configuration
  std::string gripper_type_;
  double current_gripper_position_m_;
  double gripper_command_m_;
  
  // Firmware info
  bool firmware_version_detected_;
  bool firmware_new_;  // true for v6+
  std::string firmware_version_;
  
  // Thread safety
  std::mutex data_mutex_;
  
  // State query tracking
  rclcpp::Time last_state_query_time_;
  double state_query_period_sec_;
  
  // Speed control (for V6+ firmware)
  double default_speed_rad_s_;  // Default speed in rad/s (~20 deg/s = 0.349 rad/s)
  
  // Command change detection (ROS1-style: only send on change)
  std::vector<double> last_sent_positions_;
  double last_sent_gripper_;
  rclcpp::Time last_write_time_;
  double command_change_threshold_;  // Minimum change to trigger command send
  double min_write_period_;          // Minimum time between writes during motion (500Hz)
  double min_write_period_idle_;      // Minimum time between writes when idle (100Hz)
  bool has_sent_initial_command_;
  
  // Helper functions
  void process_serial_data();
  void parse_servo_states_frame(const std::vector<uint8_t>& data_payload);
  void parse_gripper_state_frame(const std::vector<uint8_t>& data_payload);
  void parse_error_frame(const std::vector<uint8_t>& payload);
  void parse_version_frame(const std::vector<uint8_t>& full_frame);
  void send_firmware_query();
  void send_state_query();
  void set_speed(double speed_rad_s);
  
  uint16_t rad_to_hardware_value(double angle_rad);
  uint16_t rad_to_hardware_value_grip(double angle_deg);
  double hardware_value_to_rad(uint16_t hw_value);
  double hardware_value_to_rad_grip(uint16_t hw_value);
  uint8_t calculate_checksum(const std::vector<uint8_t>& frame_data);
  std::vector<uint8_t> generate_simple_frame(uint8_t command, uint8_t data, bool use_checksum);
  
  // Protocol constants
  static constexpr uint8_t FRAME_START_BYTE = 0xAA;
  static constexpr uint8_t FRAME_END_BYTE = 0xFF;
  static constexpr uint8_t CMD_SERVO_CONTROL = 0x04;
  static constexpr uint8_t CMD_GRIPPER_CONTROL = 0x02;
  static constexpr uint8_t CMD_ZERO_CAL = 0x03;
  static constexpr uint8_t CMD_DEMO_CONTROL = 0x13;
  static constexpr uint8_t CMD_SPEED = 0x05;
  static constexpr uint8_t CMD_VERSION_QUERY = 0x0A;
  static constexpr uint8_t FEEDBACK_GRIPPER_STATE = 0x02;
  static constexpr uint8_t FEEDBACK_SERVO_STATE = 0x04;
  static constexpr uint8_t FEEDBACK_GRIPPER_STATE_V6 = 0x12;
  static constexpr uint8_t FEEDBACK_SERVO_STATE_V6 = 0x14;
  static constexpr uint8_t FEEDBACK_SERVO_STATE_EXT = 0x06;
  static constexpr uint8_t FEEDBACK_ERROR = 0xEE;
  static constexpr uint8_t FEEDBACK_VERSION = 0x0A;
};

}  // namespace alicia_d_driver

#endif  // ALICIA_D_DRIVER__ALICIA_D_HARDWARE_INTERFACE_HPP_

