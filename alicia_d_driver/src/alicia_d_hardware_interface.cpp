#include "alicia_d_driver/alicia_d_hardware_interface.hpp"

#include <cmath>
#include <numeric>
#include <algorithm>
#include <string>
#include <vector>

namespace alicia_d_driver
{

CallbackReturn AliciaDHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  // Get parameters from URDF
  port_ = info_.hardware_parameters["port"];
  baud_rate_ = std::stoi(info_.hardware_parameters["baud_rate"]);
  debug_mode_ = info_.hardware_parameters.count("debug_mode") ? 
                (info_.hardware_parameters["debug_mode"] == "true") : false;
  servo_count_ = info_.hardware_parameters.count("servo_count") ? 
                 std::stoi(info_.hardware_parameters["servo_count"]) : 9;
  gripper_type_ = info_.hardware_parameters.count("gripper_type") ? 
                  info_.hardware_parameters["gripper_type"] : "50mm";
  firmware_version_ = info_.hardware_parameters.count("firmware_version") ? 
                      info_.hardware_parameters["firmware_version"] : "auto";

  // Initialize state and command vectors
  hw_positions_state_.resize(info_.joints.size(), 0.0);
  hw_positions_command_.resize(info_.joints.size(), 0.0);

  // Setup joint to servo mappings
  joint_to_servo_map_index_ = {0, 0, 1, 1, 2, 2, 3, 4, 5};
  joint_to_servo_map_direction_ = {1.0, 1.0, 1.0, -1.0, 1.0, -1.0, 1.0, 1.0, 1.0};
  servo_to_joint_map_index_ = {0, -1, 1, -1, 2, -1, 3, 4, 5}; // -1 means ignore
  servo_to_joint_map_direction_ = {1.0, 0, 1.0, 0, 1.0, 0, 1.0, 1.0, 1.0};

  firmware_version_detected_ = false;
  firmware_new_ = false;
  current_gripper_position_m_ = 0.0;
  gripper_command_m_ = 0.0;
  
  // Initialize state query timing (query state every 20ms = 50Hz)
  state_query_period_sec_ = 0.02;
  last_state_query_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

  // Initialize ROS1-style command change detection
  last_sent_positions_.resize(info_.joints.size() - 1, 0.0);  // -1 for gripper
  last_sent_gripper_ = 0.0;
  last_write_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  command_change_threshold_ = 0.0001;  // 0.0001 rad (~0.006 deg) - very sensitive for smooth trajectory execution
  min_write_period_ = 0.0002;           // 2ms = 500Hz max during motion (matches ros2_control rate)
  min_write_period_idle_ = 0.010;     // 10ms = 100Hz when idle (prevents unnecessary writes when stationary)
  has_sent_initial_command_ = false;

  RCLCPP_INFO(rclcpp::get_logger("AliciaDHardwareInterface"), 
              "Initialized hardware interface (ROS1-style: send commands only on change)");
  RCLCPP_INFO(rclcpp::get_logger("AliciaDHardwareInterface"), 
              "Port: %s, Baud: %d, Change threshold: %.4f rad", 
              port_.c_str(), baud_rate_, command_change_threshold_);

  return CallbackReturn::SUCCESS;
}

CallbackReturn AliciaDHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("AliciaDHardwareInterface"), "Configuring hardware interface...");
  
  // Create serial communicator
  communicator_ = std::make_unique<SerialCommunicator>(port_, baud_rate_, debug_mode_);
  
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> AliciaDHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_state_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> AliciaDHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_command_[i]));
  }

  return command_interfaces;
}

CallbackReturn AliciaDHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("AliciaDHardwareInterface"), "Activating hardware interface...");
  
  // Connect to serial port
  if (!communicator_->connect())
  {
    RCLCPP_ERROR(rclcpp::get_logger("AliciaDHardwareInterface"), 
                 "Failed to connect to serial port: %s", port_.c_str());
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("AliciaDHardwareInterface"), 
              "Connected to robot. Enabling full torque mode.");

  // Enable full torque mode
  auto frame = generate_simple_frame(CMD_DEMO_CONTROL, 0x01, true);
  communicator_->write_raw_frame(frame);

  // Query firmware version if needed
  if (firmware_version_.empty() || firmware_version_ == "auto")
  {
    RCLCPP_INFO(rclcpp::get_logger("AliciaDHardwareInterface"), 
                "Firmware version not specified, attempting auto-detection...");
    send_firmware_query();
  }
  else
  {
    firmware_version_detected_ = true;
    firmware_new_ = (!firmware_version_.empty() && firmware_version_[0] >= '6');
  }

  // Initialize command to current state
  hw_positions_command_ = hw_positions_state_;

  return CallbackReturn::SUCCESS;
}

CallbackReturn AliciaDHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("AliciaDHardwareInterface"), "Deactivating hardware interface...");
  
  // Disconnect from serial port
  if (communicator_)
  {
    communicator_->disconnect();
  }

  return CallbackReturn::SUCCESS;
}

return_type AliciaDHardwareInterface::read(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  if (!communicator_ || !communicator_->is_connected())
  {
    return return_type::ERROR;
  }

  // Process incoming serial data - updates hw_positions_state_ when robot sends feedback
  process_serial_data();
  
  // ROS1-style heartbeat: Always have valid state data
  // hw_positions_state_ is updated by process_serial_data() when robot sends feedback
  // If no new data, we still publish last known state (this is what makes RViz update smoothly)
  
  static int read_count = 0;
  static int last_packet_count = 0;
  read_count++;
  
  // Periodic logging to monitor communication
  if (read_count % 5000 == 0)
  {
    extern int g_total_packets_received;
    if (g_total_packets_received > last_packet_count)
    {
      int new_packets = g_total_packets_received - last_packet_count;
      RCLCPP_INFO(rclcpp::get_logger("AliciaDHardwareInterface"), 
                  "Robot responding: %d packets/sec | J1=%.3f rad", 
                  new_packets, hw_positions_state_.size() > 0 ? hw_positions_state_[0] : 0.0);
      last_packet_count = g_total_packets_received;
    }
    else
    {
      RCLCPP_WARN(rclcpp::get_logger("AliciaDHardwareInterface"), 
                  "NO packets from robot! Check connection. Total: %d", 
                  g_total_packets_received);
    }
  }

  // State interfaces are always available through hw_positions_state_
  // ros2_control will read them every cycle and publish to /joint_states
  return return_type::OK;
}

return_type AliciaDHardwareInterface::write(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  if (!communicator_ || !communicator_->is_connected())
  {
    return return_type::ERROR;
  }

  // For SMOOTH trajectory execution: Send ALL commands during motion at full 500Hz rate
  // ros2_control calls write() at 500Hz - we must send every command for smooth motion
  
  std::lock_guard<std::mutex> lock(data_mutex_);
  
  // Check if command has changed (use extremely sensitive threshold to catch all changes)
  bool command_changed = !has_sent_initial_command_;  // Always send first command
  bool has_any_change = false;
  
  // Check all joints (including gripper) for ANY change
  for (size_t i = 0; i < hw_positions_command_.size(); ++i)
  {
    double current_val = hw_positions_command_[i];
    double last_val = (i < last_sent_positions_.size()) ? last_sent_positions_[i] : 
                      ((i == 6) ? last_sent_gripper_ : 0.0);  // Gripper uses separate variable
    
    double diff = std::abs(current_val - last_val);
    // Extremely sensitive: catch ANY numeric difference (even floating point precision changes)
    if (diff > 1e-9)
    {
      command_changed = true;
      has_any_change = true;
      break;  // Found a change, no need to check others
    }
  }

  // During active motion: send at full ros2_control rate (500Hz = 2ms period)
  // During idle: only send if changed and enough time passed
  if (has_any_change)
  {
    // Active motion detected - send immediately (only respect hardware min period of 2ms)
    if ((time - last_write_time_).seconds() < min_write_period_)
    {
      return return_type::OK;  // Hardware serial rate limit (500Hz max)
    }
  }
  else
  {
    // No change detected - idle mode
    if (has_sent_initial_command_)
    {
      // Check if enough time has passed for idle check
      if ((time - last_write_time_).seconds() < min_write_period_idle_)
      {
        return return_type::OK;
      }
      // Still no change after idle period - skip
      return return_type::OK;
    }
    // Haven't sent initial command yet - send it
  }

  // Command HAS changed or initial command - send it!
  static int write_count = 0;
  write_count++;
  
  if (write_count % 100 == 0 || !has_sent_initial_command_)  // Log first and every 50 commands during motion
  {
    RCLCPP_INFO(rclcpp::get_logger("AliciaDHardwareInterface"), 
                "Sending command to robot (total: %d, motion: %s)", 
                write_count, has_any_change ? "yes" : "no");
  }
  
  // Update last sent values (lock already held from above)
  // Update joint positions
  for (size_t i = 0; i < hw_positions_command_.size() && i < last_sent_positions_.size(); ++i)
  {
    last_sent_positions_[i] = hw_positions_command_[i];
  }
  
  // Update gripper if present
  if (hw_positions_command_.size() > 6)
  {
    last_sent_gripper_ = hw_positions_command_[6];
  }
  
  last_write_time_ = time;
  has_sent_initial_command_ = true;

  // Build and send servo frame
  size_t frame_size = servo_count_ * 2 + 5;
  std::vector<uint8_t> servo_frame(frame_size);
  servo_frame[0] = FRAME_START_BYTE;
  servo_frame[1] = CMD_SERVO_CONTROL;
  servo_frame[2] = servo_count_ * 2;

  for (int i = 0; i < servo_count_; ++i)
  {
    uint16_t hw_val = 2048;
    if (static_cast<size_t>(i) < joint_to_servo_map_index_.size())
    {
      int joint_idx = joint_to_servo_map_index_[i];
      double direction = joint_to_servo_map_direction_[i];
      
      // Find the joint in our joints list
      if (joint_idx < 6 && static_cast<size_t>(joint_idx) < hw_positions_command_.size())
      {
        hw_val = rad_to_hardware_value(hw_positions_command_[joint_idx] * direction);
      }
    }
    size_t idx = 3 + i * 2;
    servo_frame[idx] = hw_val & 0xFF;
    servo_frame[idx + 1] = (hw_val >> 8) & 0xFF;
  }

  servo_frame[frame_size - 2] = calculate_checksum(servo_frame);
  servo_frame[frame_size - 1] = FRAME_END_BYTE;
  communicator_->write_raw_frame(servo_frame);

  // Build and send gripper frame (if gripper joint exists)
  if (hw_positions_command_.size() > 6)
  {
    // Convert gripper position (meters) to hardware value (0-100 degrees)
    const double stroke_m = (gripper_type_ == "100mm") ? 0.05 : 0.025;
    double m = std::max(0.0, std::min(stroke_m, hw_positions_command_[6]));
    double gripper_value = 100.0 - ((stroke_m > 1e-6 ? m / stroke_m : 0.0) * 100.0);

    if (firmware_new_)
    {
      std::vector<uint8_t> gripper_frame(11);
      gripper_frame[0] = FRAME_START_BYTE;
      gripper_frame[1] = CMD_GRIPPER_CONTROL;
      gripper_frame[2] = 6;
      gripper_frame[3] = 1;
      uint16_t target = rad_to_hardware_value_grip(gripper_value);
      gripper_frame[4] = 3400 & 0xFF;
      gripper_frame[5] = (3400 >> 8) & 0xFF;
      gripper_frame[6] = target & 0xFF;
      gripper_frame[7] = (target >> 8) & 0xFF;
      gripper_frame[8] = 254;
      gripper_frame[9] = calculate_checksum(gripper_frame);
      gripper_frame[10] = FRAME_END_BYTE;
      communicator_->write_raw_frame(gripper_frame);
    }
    else
    {
      std::vector<uint8_t> gripper_frame(8);
      gripper_frame[0] = FRAME_START_BYTE;
      gripper_frame[1] = CMD_GRIPPER_CONTROL;
      gripper_frame[2] = 3;
      gripper_frame[3] = 1;
      uint16_t target = rad_to_hardware_value_grip(gripper_value);
      gripper_frame[4] = target & 0xFF;
      gripper_frame[5] = (target >> 8) & 0xFF;
      gripper_frame[6] = calculate_checksum(gripper_frame);
      gripper_frame[7] = FRAME_END_BYTE;
      communicator_->write_raw_frame(gripper_frame);
    }
  }

  return return_type::OK;
}

// Global counter for cross-function tracking
int g_total_packets_received = 0;

void AliciaDHardwareInterface::process_serial_data()
{
  std::vector<uint8_t> packet;
  
  static int packet_count = 0;
  static int unknown_count = 0;
  static int servo_count_received = 0;
  static int gripper_count_received = 0;
  
  // Process all available packets in the queue
  while (communicator_->get_packet(packet))
  {
    if (packet.empty())
    {
      continue;
    }

    packet_count++;
    g_total_packets_received++;
    
    // The first byte of the payload is the command ID
    uint8_t command_id = packet[0];

    // Log periodically for debugging
    if (packet_count % 100 == 0)
    {
      RCLCPP_INFO(rclcpp::get_logger("AliciaDHardwareInterface"), 
                  "Packets: total=%d, servo=%d, gripper=%d, unknown=%d", 
                  packet_count, servo_count_received, gripper_count_received, unknown_count);
    }

    // Create a sub-vector that contains the actual data payload for the command
    std::vector<uint8_t> data_payload;
    if (packet.size() > 1)
    {
      data_payload.assign(packet.begin() + 1, packet.end());
    }

    switch (command_id)
    {
      case FEEDBACK_SERVO_STATE:
      case FEEDBACK_SERVO_STATE_V6:
        servo_count_received++;
        parse_servo_states_frame(data_payload);
        break;
      case FEEDBACK_GRIPPER_STATE:
      case FEEDBACK_GRIPPER_STATE_V6:
        gripper_count_received++;
        parse_gripper_state_frame(data_payload);
        break;
      case FEEDBACK_ERROR:
        parse_error_frame(data_payload);
        break;
      case FEEDBACK_VERSION:
        parse_version_frame(data_payload);
        break;
      default:
        unknown_count++;
        // Log first few unknown frames for debugging
        if (unknown_count <= 10)
        {
          RCLCPP_WARN(rclcpp::get_logger("AliciaDHardwareInterface"), 
                      "Unknown command ID: 0x%02X, packet_size=%zu", 
                      command_id, packet.size());
        }
        break;
    }
  }
}

void AliciaDHardwareInterface::parse_servo_states_frame(const std::vector<uint8_t>& data_payload)
{
  if (data_payload.empty())
  {
    return;
  }

  uint8_t data_byte_count = data_payload[0];
  if (data_payload.size() < (size_t)data_byte_count + 1)
  {
    return;
  }

  int servos_in_frame = data_byte_count / 2;

  std::lock_guard<std::mutex> lock(data_mutex_);

  static int parse_count = 0;
  parse_count++;
  if (parse_count % 50 == 0)  // Log every 50 frames (~1 second at 50Hz)
  {
    RCLCPP_INFO(rclcpp::get_logger("AliciaDHardwareInterface"), 
                "Receiving servo states: %d servos, J1=%.3f rad", 
                servos_in_frame, hw_positions_state_.size() > 0 ? hw_positions_state_[0] : 0.0);
  }

  for (int i = 0; i < servos_in_frame && i < servo_count_; ++i)
  {
    size_t data_idx = 1 + i * 2;
    if (data_idx + 1 >= data_payload.size()) break;
    
    uint16_t hw_val = data_payload[data_idx] | (data_payload[data_idx + 1] << 8);
    double rad_val = hardware_value_to_rad(hw_val);

    if (static_cast<size_t>(i) < servo_to_joint_map_index_.size())
    {
      int joint_idx = servo_to_joint_map_index_[i];
      if (joint_idx != -1 && static_cast<size_t>(joint_idx) < hw_positions_state_.size())
      {
        hw_positions_state_[joint_idx] = rad_val * servo_to_joint_map_direction_[i];
      }
    }
  }
}

void AliciaDHardwareInterface::parse_gripper_state_frame(const std::vector<uint8_t>& data_payload)
{
  if (data_payload.size() < 8)
  {
    return;
  }

  uint16_t gripper_hw_val = data_payload[2] | (data_payload[3] << 8);
  
  std::lock_guard<std::mutex> lock(data_mutex_);
  current_gripper_position_m_ = hardware_value_to_rad_grip(gripper_hw_val);
  
  if (hw_positions_state_.size() > 6)
  {
    hw_positions_state_[6] = current_gripper_position_m_;
  }
}

void AliciaDHardwareInterface::parse_error_frame(const std::vector<uint8_t>& payload)
{
  if (payload.size() < 2)
  {
    return;
  }
  uint8_t error_type = payload[0];
  uint8_t error_param = payload[1];
  RCLCPP_ERROR(rclcpp::get_logger("AliciaDHardwareInterface"), 
               "Received Error Frame from Hardware: Type=0x%02X, Param=0x%02X", 
               error_type, error_param);
}

void AliciaDHardwareInterface::parse_version_frame(const std::vector<uint8_t>& data_payload)
{
  if (firmware_version_detected_) return;
  if (data_payload.size() < 4) return;
  
  uint8_t major = data_payload[1];
  uint8_t minor = data_payload[2];
  uint8_t patch = data_payload[3];
  
  char buf[16];
  snprintf(buf, sizeof(buf), "%d.%d.%d", major, minor, patch);
  firmware_version_ = std::string(buf);
  firmware_new_ = (major >= 6);
  firmware_version_detected_ = true;
  
  RCLCPP_INFO(rclcpp::get_logger("AliciaDHardwareInterface"), 
              "Firmware version detected: %s (new=%s)", 
              firmware_version_.c_str(), firmware_new_ ? "true" : "false");
}

void AliciaDHardwareInterface::send_firmware_query()
{
  if (!communicator_ || !communicator_->is_connected()) return;
  
  std::vector<uint8_t> frame(6);
  frame[0] = FRAME_START_BYTE;
  frame[1] = CMD_VERSION_QUERY;
  frame[2] = 0x01;
  frame[3] = 0x00;
  frame[4] = 0x00;
  frame[5] = FRAME_END_BYTE;
  communicator_->write_raw_frame(frame);
}

void AliciaDHardwareInterface::send_state_query()
{
  if (!communicator_ || !communicator_->is_connected()) return;
  
  // Send a query for current servo states
  // Command 0x05 is typically a read/query command in many protocols
  // If this doesn't work, the robot might auto-send state or we need a different command
  std::vector<uint8_t> frame(6);
  frame[0] = FRAME_START_BYTE;
  frame[1] = 0x05;  // State query command (you may need to adjust based on robot protocol)
  frame[2] = 0x01;  // Data length
  frame[3] = 0x00;  // Query all servos
  frame[4] = 0x00;  // Checksum (0 for query)
  frame[5] = FRAME_END_BYTE;
  communicator_->write_raw_frame(frame);
}

uint16_t AliciaDHardwareInterface::rad_to_hardware_value(double angle_rad)
{
  double angle_deg = angle_rad * 180.0 / M_PI;
  angle_deg = std::max(-180.0, std::min(180.0, angle_deg));
  int value = static_cast<int>((angle_deg + 180.0) / 360.0 * 4096.0);
  return std::max(0, std::min(4095, value));
}

uint16_t AliciaDHardwareInterface::rad_to_hardware_value_grip(double angle_deg)
{
  angle_deg = std::max(0.0, std::min(100.0, angle_deg));
  int hardware_value = static_cast<int>(angle_deg * 8.52 + 2048.0);
  return std::max(2048, std::min(2900, hardware_value));
}

double AliciaDHardwareInterface::hardware_value_to_rad(uint16_t hw_value)
{
  hw_value = std::max(0, std::min(4095, (int)hw_value));
  double angle_deg = -180.0 + (static_cast<double>(hw_value) / 4095.0) * 360.0;
  return angle_deg * M_PI / 180.0;
}

double AliciaDHardwareInterface::hardware_value_to_rad_grip(uint16_t hw_value)
{
  hw_value = std::max(2048, std::min(2900, (int)hw_value));
  double angle_deg = (static_cast<double>(hw_value) - 2048.0) / 8.52;
  double open_pct = std::max(0.0, std::min(100.0, angle_deg));
  const double stroke_m = (gripper_type_ == "100mm") ? 0.05 : 0.025;
  double meters = (1.0 - (open_pct / 100.0)) * stroke_m;
  return meters;
}

uint8_t AliciaDHardwareInterface::calculate_checksum(const std::vector<uint8_t>& frame_data)
{
  if (frame_data.size() < 4)
  {
    return 0;
  }
  
  const uint8_t payload_len = frame_data[2];

  if (frame_data.size() < (size_t)3 + payload_len)
  {
    return 0;
  }

  int sum = std::accumulate(frame_data.begin() + 3, 
                            frame_data.begin() + 3 + payload_len, 
                            0);

  return static_cast<uint8_t>(sum % 2);
}

std::vector<uint8_t> AliciaDHardwareInterface::generate_simple_frame(
  uint8_t command, uint8_t data, bool use_checksum)
{
  std::vector<uint8_t> frame(6);
  frame[0] = FRAME_START_BYTE;
  frame[1] = command;
  frame[2] = 0x01;
  frame[3] = data & 0xFF;

  if (use_checksum)
  {
    frame[4] = data % 2;
  }
  else
  {
    frame[4] = 0x00;
  }

  frame[5] = FRAME_END_BYTE;
  return frame;
}

}  // namespace alicia_d_driver

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(alicia_d_driver::AliciaDHardwareInterface, hardware_interface::SystemInterface)

