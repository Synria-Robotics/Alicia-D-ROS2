#include "alicia_d_driver/alicia_d_hardware_interface.hpp"

#include <cmath>
#include <numeric>
#include <algorithm>
#include <string>
#include <vector>
#include <chrono>
#include <thread>

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
  
  // Speed control parameter (for V6+ firmware, default ~20 deg/s = 0.349 rad/s)
  default_speed_rad_s_ = info_.hardware_parameters.count("default_speed_rad_s") ? 
                         std::stod(info_.hardware_parameters["default_speed_rad_s"]) : 0.349;

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

  // Initialize timing for rate limiting
  last_sent_positions_.resize(info_.joints.size() - 1, 0.0);  // -1 for gripper
  last_sent_gripper_ = -1.0;  // Initialize to invalid value to force first send
  last_command_gripper_ = -1.0;  // Initialize to invalid value
  last_write_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  last_gripper_send_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  command_change_threshold_ = 0.0001;  // Unused now, kept for compatibility
  min_write_period_ = 0.020;            // 20ms = 50Hz (matches Python SDK HardwareExecutor delay)
  min_write_period_idle_ = 0.010;      // Unused now
  has_sent_initial_command_ = false;

  RCLCPP_INFO(rclcpp::get_logger("AliciaDHardwareInterface"), 
              "Initialized hardware interface (rate-limited command sending)");
  RCLCPP_INFO(rclcpp::get_logger("AliciaDHardwareInterface"), 
              "Port: %s, Baud: %d, Command rate limit: %.0f Hz (matches Python SDK)", 
              port_.c_str(), baud_rate_, 1.0 / min_write_period_);

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



  // Query firmware version if needed
  if (firmware_version_.empty() || firmware_version_ == "auto")
  {
    RCLCPP_INFO(rclcpp::get_logger("AliciaDHardwareInterface"), 
                "Firmware version not specified, attempting auto-detection...");
    send_firmware_query();
    // Wait and process responses to get firmware version
    // Try multiple times as robot may need time to respond
    for (int i = 0; i < 40 && !firmware_version_detected_; ++i)  // Wait up to 2 seconds
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      process_serial_data();  // Process any incoming firmware version response
    }
    
    // If still not detected, try sending query again
    if (!firmware_version_detected_)
    {
      RCLCPP_INFO(rclcpp::get_logger("AliciaDHardwareInterface"), 
                  "First query timeout, sending version query again...");
      send_firmware_query();
      for (int i = 0; i < 20 && !firmware_version_detected_; ++i)  // Wait another 1 second
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        process_serial_data();
      }
    }
  }
  else
  {
    firmware_version_detected_ = true;
    // Parse version string to determine if V6+
    // Version format: "X.Y.Z" where X is major version
    if (!firmware_version_.empty() && firmware_version_[0] >= '6')
    {
      firmware_new_ = true;
    }
    else
    {
      firmware_new_ = false;
    }
    RCLCPP_INFO(rclcpp::get_logger("AliciaDHardwareInterface"), 
                "Using specified firmware version: %s (V6+=%s)", 
                firmware_version_.c_str(), firmware_new_ ? "true" : "false");
  }

  // Set speed for V6+ firmware (critical for smooth motion!)
  if (firmware_new_)
  {
    RCLCPP_INFO(rclcpp::get_logger("AliciaDHardwareInterface"), 
                "V6+ firmware detected, setting speed: %.1f deg/s (%.3f rad/s)", 
                default_speed_rad_s_ * 180.0 / M_PI, default_speed_rad_s_);
    set_speed(default_speed_rad_s_);
  }
  else if (firmware_version_detected_)
  {
    RCLCPP_INFO(rclcpp::get_logger("AliciaDHardwareInterface"), 
                "V5 firmware detected, speed control not available");
  }
  else
  {
    RCLCPP_WARN(rclcpp::get_logger("AliciaDHardwareInterface"), 
                "Firmware version not detected. Defaulting to V5 behavior (no speed control).");
    RCLCPP_WARN(rclcpp::get_logger("AliciaDHardwareInterface"), 
                "If this is incorrect, specify firmware_version parameter explicitly.");
    // Default to V5 (older firmware) if detection fails
    firmware_new_ = false;
    // Do NOT set speed for V5 firmware
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
  

  static int read_count = 0;
  static int last_packet_count = 0;
  read_count++;
  
  // Periodic logging to monitor communication (DEBUG level only)
  if (read_count % 5000 == 0)
  {
    extern int g_total_packets_received;
    if (g_total_packets_received > last_packet_count)
    {
      int new_packets = g_total_packets_received - last_packet_count;
      RCLCPP_DEBUG(rclcpp::get_logger("AliciaDHardwareInterface"), 
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
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  if (!communicator_ || !communicator_->is_connected())
  {
    return return_type::ERROR;
  }

  std::lock_guard<std::mutex> lock(data_mutex_);
  
  // Rate limit command sending to match Python SDK behavior (50Hz = 20ms)
  double elapsed = (time - last_write_time_).seconds();
  if (elapsed < min_write_period_)
  {
    return return_type::OK;  // Skip this write, rate limit to 50Hz
  }
  
  last_write_time_ = time;
  
  static int write_count = 0;
  static rclcpp::Time last_log_time = rclcpp::Time(0, 0, RCL_ROS_TIME);
  write_count++;
  
  // Initialize on first call
  if (last_log_time.seconds() == 0.0) {
    last_log_time = time;
  }
  
  // Log frequency and command values periodically (DEBUG level only)
  double time_since_log = (time - last_log_time).seconds();
  if (time_since_log >= 1.0)  // Every second
  {
    double actual_rate = write_count / time_since_log;
    RCLCPP_DEBUG(rclcpp::get_logger("AliciaDHardwareInterface"), 
                "Write rate: %.1f Hz (expected: %.1f Hz), Commands sent: %d | J1=%.3f rad", 
                actual_rate, 1.0 / period.seconds(), write_count,
                hw_positions_command_.size() > 0 ? hw_positions_command_[0] : 0.0);
    write_count = 0;
    last_log_time = time;
  }

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

  // Build and send gripper frame (if gripper joint exists) - only on change
  if (hw_positions_command_.size() > 6)
  {
    // Convert gripper position (meters) to 0-100 value
    // Matching ROS1 driver node: URDF 0 = open, URDF stroke_m = closed
    // Python SDK: 0 = closed, 100 = open
    // Conversion: URDF meters -> 0-100 value (0=closed, 100=open)
    const double stroke_m = (gripper_type_ == "100mm") ? 0.05 : 0.025;
    double m = std::max(0.0, std::min(stroke_m, hw_positions_command_[6]));
    // ROS1 formula: gripper_value = 100 - (m / stroke_m * 100)
    // URDF 0 (open) -> gripper_value 100 (open)
    // URDF stroke_m (closed) -> gripper_value 0 (closed)
    double gripper_value = 100.0 - ((stroke_m > 1e-6 ? m / stroke_m : 0.0) * 100.0);

    // Detect trajectory start: large change OR pause indicates new trajectory
    // This ensures waypoints in new trajectories are not skipped even if close to previous end position
    const double trajectory_start_threshold = 2.0;  // 2% of range indicates new trajectory
    const double trajectory_pause_threshold_sec = 0.1;  // 100ms pause indicates new trajectory
    bool is_new_trajectory = false;
    
    if (last_command_gripper_ >= 0.0)  // Valid previous command
    {
      double change_from_last_cmd = std::abs(gripper_value - last_command_gripper_);
      
      // Large jump (>= 2%) between consecutive commands indicates a new trajectory has started
      if (change_from_last_cmd >= trajectory_start_threshold)
      {
        is_new_trajectory = true;
      }
      // If no gripper command sent for a while (>100ms), likely a new trajectory started
      else if (last_gripper_send_time_.seconds() > 0.0)
      {
        double time_since_last_send = (time - last_gripper_send_time_).seconds();
        if (time_since_last_send >= trajectory_pause_threshold_sec)
        {
          is_new_trajectory = true;
        }
      }
      
      if (is_new_trajectory)
      {
        // Reset last_sent_gripper_ to force sending all waypoints in new trajectory
        last_sent_gripper_ = -1.0;
      }
    }
    
    // Update last commanded value
    last_command_gripper_ = gripper_value;

    // Only send gripper command if value changed significantly (threshold: 0.001 = 0.1%)
    // or if this is the first command (last_sent_gripper_ is invalid)
    // or if it's a new trajectory start
    const double gripper_threshold = 0.001;  
    bool gripper_changed = (last_sent_gripper_ < 0.0) ||  // First send
                           is_new_trajectory ||           // New trajectory start
                           (std::abs(gripper_value - last_sent_gripper_) >= gripper_threshold);
    if (gripper_changed)
    {
      // Update last sent value and timestamp
      last_sent_gripper_ = gripper_value;
      last_gripper_send_time_ = time;

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
    
    // Full frame format: AA CMD LEN DATA... CHK FF
    // packet[0] = AA, packet[1] = CMD, packet[2] = LEN
    if (packet.empty() || packet[0] != FRAME_START_BYTE)
    {
      RCLCPP_WARN(rclcpp::get_logger("AliciaDHardwareInterface"), 
                  "Invalid packet: missing frame start byte");
      continue;
    }
    
    uint8_t command_id = packet[1];  // Command is at index 1 (after AA)
    


    if (packet_count % 100 == 0)
    {
      RCLCPP_DEBUG(rclcpp::get_logger("AliciaDHardwareInterface"), 
                  "Packets: total=%d, servo=%d, gripper=%d, unknown=%d", 
                  packet_count, servo_count_received, gripper_count_received, unknown_count);
    }

    // Extract data payload: includes LEN byte to match parse function expectations
    // Frame format: AA CMD LEN DATA... CHK FF
    // Parse functions expect: LEN DATA...
    std::vector<uint8_t> data_payload;
    if (packet.size() >= 4)
    {
      uint8_t data_len = packet[2];
      if (packet.size() >= (size_t)3 + data_len + 2)  // AA CMD LEN DATA CHK FF
      {
        // Payload includes LEN byte: from index 2 (LEN) to index 2+LEN (before checksum)
        data_payload.assign(packet.begin() + 2, packet.begin() + 3 + data_len);
      }
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
        parse_version_frame(packet);  
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
    RCLCPP_DEBUG(rclcpp::get_logger("AliciaDHardwareInterface"), 
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

void AliciaDHardwareInterface::parse_version_frame(const std::vector<uint8_t>& full_frame)
{
  if (firmware_version_detected_) return;
  
  // Full frame format: AA CMD LEN major minor patch ... CHK FF
  // Python SDK accesses frame[3], frame[4], frame[5] for version
  // So: frame[0]=AA, frame[1]=CMD, frame[2]=LEN, frame[3]=major, frame[4]=minor, frame[5]=patch
  if (full_frame.size() < 6) 
  {
    RCLCPP_WARN(rclcpp::get_logger("AliciaDHardwareInterface"), 
                "Version frame too short: %zu bytes (expected at least 6)", full_frame.size());
    return;
  }
  
  // Extract version from indices 3, 4, 5 (matching Python SDK)
  uint8_t major = full_frame[3];
  uint8_t minor = full_frame[4];
  uint8_t patch = full_frame[5];
  
  char buf[16];
  snprintf(buf, sizeof(buf), "%d.%d.%d", major, minor, patch);
  firmware_version_ = std::string(buf);
  firmware_new_ = (major >= 6);
  firmware_version_detected_ = true;
  
  RCLCPP_INFO(rclcpp::get_logger("AliciaDHardwareInterface"), 
              "Firmware version detected: %s (V6+=%s)", 
              firmware_version_.c_str(), firmware_new_ ? "true" : "false");
}

void AliciaDHardwareInterface::send_firmware_query()
{
  if (!communicator_ || !communicator_->is_connected()) return;
  
  // Version query frame format matching Python SDK: AA 0A 01 00 00 FF
  // Frame: AA CMD LEN DATA CHK FF
  // For simple query: LEN=1, DATA=0x00, CHK=0x00 (sum of payload=0, mod 2 = 0)
  std::vector<uint8_t> frame(6);
  frame[0] = FRAME_START_BYTE;  // AA
  frame[1] = CMD_VERSION_QUERY; // 0x0A
  frame[2] = 0x01;              // Length = 1
  frame[3] = 0x00;              // Data byte
  frame[4] = 0x00;              // Checksum (0x00 mod 2 = 0)
  frame[5] = FRAME_END_BYTE;    // FF
  
  RCLCPP_DEBUG(rclcpp::get_logger("AliciaDHardwareInterface"), 
               "Sending firmware version query: AA 0A 01 00 00 FF");
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

void AliciaDHardwareInterface::set_speed(double speed_rad_s)
{
  if (!communicator_ || !communicator_->is_connected())
  {
    RCLCPP_WARN(rclcpp::get_logger("AliciaDHardwareInterface"), 
                "Cannot set speed: not connected");
    return;
  }

  // Convert rad/s to hardware speed value (1-3400)
  // Based on Python SDK: max_angle_rad_per_sec = 2*pi, max_speed_value = 3400
  const double max_angle_rad_per_sec = 2.0 * M_PI;
  const double max_speed_value = 3400.0;
  double raw_speed = (speed_rad_s / max_angle_rad_per_sec) * max_speed_value;
  int speed_value = std::max(1, std::min(3400, static_cast<int>(raw_speed)));

  // Build speed control frame: [START, CMD_SPEED, length, 0x2E, speed_bytes...]
  // Speed command format: 0x2E + speed value (2 bytes) for each of 10 servos
  std::vector<uint8_t> frame(26);  // 5 header + 0x2E + 10 servos * 2 bytes = 26
  frame[0] = FRAME_START_BYTE;
  frame[1] = CMD_SPEED;
  frame[2] = 21;  // Data length: 1 (0x2E) + 10 servos * 2 = 21
  frame[3] = 0x2E;  // Speed control byte
  
  // Fill speed value for all 10 servos
  for (int i = 0; i < 10; ++i)
  {
    size_t idx = 4 + i * 2;
    frame[idx] = speed_value & 0xFF;         // Low byte
    frame[idx + 1] = (speed_value >> 8) & 0xFF; // High byte
  }
  
  frame[24] = calculate_checksum(frame);
  frame[25] = FRAME_END_BYTE;
  
  communicator_->write_raw_frame(frame);
  
  RCLCPP_INFO(rclcpp::get_logger("AliciaDHardwareInterface"), 
              "Speed set: %.3f rad/s (%.1f deg/s) -> hardware value: %d", 
              speed_rad_s, speed_rad_s * 180.0 / M_PI, speed_value);
}

uint16_t AliciaDHardwareInterface::rad_to_hardware_value(double angle_rad)
{
  double angle_deg = angle_rad * 180.0 / M_PI;
  angle_deg = std::max(-180.0, std::min(180.0, angle_deg));
  int value = static_cast<int>((angle_deg + 180.0) / 360.0 * 4096.0);
  return std::max(0, std::min(4095, value));
}

uint16_t AliciaDHardwareInterface::rad_to_hardware_value_grip(double gripper_value)
{
  // Matching ROS1 driver node exactly
  // Input range: 0 (closed) to 100 (open)
  double value = std::max(0.0, std::min(100.0, gripper_value));
  
  // Hardware mapping: 
  // gripper_value=0 (closed) -> gripper_hw_max_ (hardware closed)
  // gripper_value=100 (open) -> 2048 (hardware open)
  // This is a REVERSE linear mapping (like Python SDK)
  constexpr double GRIPPER_HW_MIN = 2048.0;  // Fully open (common for all gripper types)
  const double gripper_hw_max = (gripper_type_ == "100mm") ? 3600.0 : 3290.0;
  const double ratio = (gripper_hw_max - GRIPPER_HW_MIN) / 100.0;
  const double hw_value = gripper_hw_max - (value * ratio);  // Reverse mapping
  const int hardware_value = static_cast<int>(std::round(hw_value));
  
  return std::max(static_cast<int>(GRIPPER_HW_MIN), std::min(static_cast<int>(gripper_hw_max), hardware_value));
}

double AliciaDHardwareInterface::hardware_value_to_rad(uint16_t hw_value)
{
  hw_value = std::max(0, std::min(4095, (int)hw_value));
  double angle_deg = -180.0 + (static_cast<double>(hw_value) / 4095.0) * 360.0;
  return angle_deg * M_PI / 180.0;
}

double AliciaDHardwareInterface::hardware_value_to_rad_grip(uint16_t hw_value)
{
  // Matching ROS1 driver node exactly
  constexpr double GRIPPER_HW_MIN = 2048.0;  // Fully open (common for all gripper types)
  const double gripper_hw_max = (gripper_type_ == "100mm") ? 3600.0 : 3290.0;
  hw_value = std::max(static_cast<int>(GRIPPER_HW_MIN), std::min(static_cast<int>(gripper_hw_max), (int)hw_value));
  // Inverse map for feedback: gripper_hw_max (closed) -> 0, 2048 (open) -> 100
  const double ratio = (gripper_hw_max - GRIPPER_HW_MIN) / 100.0;
  const double gripper_value = 100.0 - ((static_cast<double>(hw_value) - GRIPPER_HW_MIN) / ratio);
  // Convert to meters for JointState publication
  // Use gripper type to determine stroke: 100mm -> 0.05m, 50mm -> 0.025m
  const double stroke_m = (gripper_type_ == "100mm") ? 0.05 : 0.025;
  // Where gripper_value=0 (closed) -> stroke_m, gripper_value=100 (open) -> 0m
  const double gripper_m = (1.0 - gripper_value / 100.0) * stroke_m;
  return gripper_m; // Return in meters (prismatic joint)
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

