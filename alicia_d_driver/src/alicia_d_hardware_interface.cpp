#include "alicia_d_driver/alicia_d_hardware_interface.hpp"

#include <cmath>
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
  debug_mode_ = info_.hardware_parameters.count("debug_mode") ? 
                (info_.hardware_parameters["debug_mode"] == "true") : false;
  
  // Gripper type (required: "50mm" or "100mm", default "50mm")
  gripper_type_param_ = info_.hardware_parameters.count("gripper_type") ? 
                         info_.hardware_parameters["gripper_type"] : "50mm";
  
  // Speed control parameter (default ~20 deg/s = 0.349 rad/s)
  default_speed_rad_s_ = info_.hardware_parameters.count("default_speed_rad_s") ? 
                         std::stod(info_.hardware_parameters["default_speed_rad_s"]) : 0.349;

  // Initialize state and command vectors
  hw_positions_state_.resize(info_.joints.size(), 0.0);
  hw_positions_command_.resize(info_.joints.size(), 0.0);
  hw_velocities_state_.resize(info_.joints.size(), 0.0);
  hw_velocities_command_.resize(info_.joints.size(), 0.0);

  // Initialize timing for real-time control (no rate limiting needed).
  // Use steady time to match controller_manager's clock type and avoid
  // "can't subtract times with different time sources" exceptions.
  last_write_time_ = rclcpp::Time(0, 0, RCL_STEADY_TIME);
  min_write_period_ = 0.0;  // No rate limiting - send commands every cycle for real-time control

  // Initialize hardware connection status
  hardware_connected_ = false;

  RCLCPP_INFO(rclcpp::get_logger("AliciaDHardwareInterface"), 
              "Initialized hardware interface (using unified data parser control)");
  RCLCPP_INFO(rclcpp::get_logger("AliciaDHardwareInterface"), 
              "Port: %s, Real-time control enabled", 
              port_.empty() ? "(auto-detect)" : port_.c_str());

  return CallbackReturn::SUCCESS;
}

CallbackReturn AliciaDHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("AliciaDHardwareInterface"), "Configuring hardware interface...");
  
  // Create serial communicator and data parser control (matching driver node)
  communicator_ = std::make_unique<SerialCommunicator>(port_, debug_mode_);
  data_parser_control_ = std::make_unique<AliciaDDataParserControl>(
      communicator_.get(), rclcpp::get_logger("AliciaDHardwareInterface"), debug_mode_, gripper_type_param_);
  
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> AliciaDHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_state_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_state_[i]));
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
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_command_[i]));
  }

  return command_interfaces;
}

CallbackReturn AliciaDHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("AliciaDHardwareInterface"), "Activating hardware interface...");
  
  // Try to connect to serial port
  if (communicator_->connect())
  {
    hardware_connected_ = true;
    RCLCPP_INFO(rclcpp::get_logger("AliciaDHardwareInterface"), 
                "Connected to robot. Starting parsing thread and querying information.");

    // Start parsing thread (matching driver node - background thread for continuous data parsing)
    data_parser_control_->start_parsing_thread();

    // Query all information types (matching driver node)
    data_parser_control_->acquire_info("version", true, 3.0, 0.2);
    data_parser_control_->acquire_info("temperature", true, 2.0, 0.2);
    data_parser_control_->acquire_info("velocity", true, 2.0, 0.2);
    data_parser_control_->acquire_info("self_check", true, 2.0, 0.2);
    
    // Print all available information
    data_parser_control_->print_information();

    // Enable torque using data parser control
    data_parser_control_->torque_control("on");
  }
  else
  {
    hardware_connected_ = false;
    RCLCPP_WARN(rclcpp::get_logger("AliciaDHardwareInterface"), 
                "No hardware connected. Running in simulation/demo mode. "
                "Hardware interface will accept commands but they will not be sent to robot.");
  }

  // Initialize command to current state
  hw_positions_command_ = hw_positions_state_;

  return CallbackReturn::SUCCESS;
}

CallbackReturn AliciaDHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("AliciaDHardwareInterface"), "Deactivating hardware interface...");
  
  // Stop parsing thread (matching driver node) only if hardware was connected
  if (hardware_connected_ && data_parser_control_)
  {
    data_parser_control_->stop_parsing_thread();
  }
  
  // Disconnect from serial port only if hardware was connected
  if (hardware_connected_ && communicator_)
  {
    communicator_->disconnect();
  }
  
  hardware_connected_ = false;

  return CallbackReturn::SUCCESS;
}

return_type AliciaDHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // If hardware is not connected, simulate state updates (copy commands to state for simulation)
  if (!hardware_connected_ || !communicator_ || !communicator_->is_connected() || !data_parser_control_)
  {
    // In simulation mode, update state to match commands (simulate ideal robot)
    std::lock_guard<std::mutex> lock(data_mutex_);
    hw_positions_state_ = hw_positions_command_;
    // Set velocities to zero in simulation (or copy from commands if provided)
    for (size_t i = 0; i < hw_velocities_state_.size(); ++i)
    {
      if (i < hw_velocities_command_.size())
      {
        hw_velocities_state_[i] = hw_velocities_command_[i];
      }
      else
      {
        hw_velocities_state_[i] = 0.0;
      }
    }
    return return_type::OK;
  }

  // Note: Parsing happens in background thread (started in on_activate).
  // We periodically request joint data to keep state fresh (matching driver node).
  // Use a steady clock so time differences are computed with a consistent
  // time source, matching controller_manager's use of steady time.
  static rclcpp::Clock steady_clock(RCL_STEADY_TIME);
  static rclcpp::Time last_joint_request(0, 0, RCL_STEADY_TIME);
  rclcpp::Time now = steady_clock.now();
  if ((now - last_joint_request).seconds() >= 0.05)  // Request at 20 Hz (matching driver node)
  {
    data_parser_control_->acquire_info("joint", false);
    last_joint_request = now;
  }
  
  // Update state from parser (data is parsed by background thread)
  auto joint_state = data_parser_control_->get_joint_state();
  auto velocity_data = data_parser_control_->get_velocity_data();
  
  if (joint_state.has_value())
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    // Update joint positions (first 6 joints)
    for (size_t i = 0; i < 6 && i < joint_state->angles.size() && i < hw_positions_state_.size(); ++i)
    {
      hw_positions_state_[i] = joint_state->angles[i];
    }
    
    // Update gripper position (convert from 0-1000 to meters)
    if (hw_positions_state_.size() > 6)
    {
      hw_positions_state_[6] = data_parser_control_->gripper_value_to_position(joint_state->gripper);
    }
    
    // Update velocities if available
    if (velocity_data.has_value() && velocity_data->velocities.size() >= 6)
    {
      // Convert from deg/s to rad/s for first 6 joints
      for (size_t i = 0; i < 6 && i < velocity_data->velocities.size() && i < hw_velocities_state_.size(); ++i)
      {
        hw_velocities_state_[i] = velocity_data->velocities[i] * M_PI / 180.0;
      }
      // Gripper velocity (if available, otherwise 0)
      if (hw_velocities_state_.size() > 6)
      {
        hw_velocities_state_[6] = 0.0;  // Gripper velocity not typically reported
      }
    }
  }

  return return_type::OK;
}

return_type AliciaDHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // If hardware is not connected, simulate command acceptance (state will be updated in read())
  if (!hardware_connected_ || !communicator_ || !communicator_->is_connected() || !data_parser_control_)
  {
    // In simulation mode, commands are accepted but not sent to hardware
    // The read() method will copy commands to state to simulate movement
    return return_type::OK;
  }

  // Real-time control: send commands every cycle (no rate limiting)
  // The robot hardware can handle high-frequency commands for smooth real-time control
  
  std::lock_guard<std::mutex> lock(data_mutex_);
  
  // Extract joint angles (first 6 joints)
  std::vector<double> joint_angles;
  for (size_t i = 0; i < 6 && i < hw_positions_command_.size(); ++i)
  {
    joint_angles.push_back(hw_positions_command_[i]);
  }
  
  // Extract gripper position and convert to value (0-1000)
  // -1.0 means use current (matching Python SDK None)
  double gripper_value = -1.0;
  if (hw_positions_command_.size() > 6)
  {
    gripper_value = data_parser_control_->gripper_position_to_value(hw_positions_command_[6]);
  }
  
  // Extract single speed value from velocities (matching Python SDK: single speed for all joints)
  // Use the maximum absolute velocity if provided, otherwise use default
  bool has_velocity_command = false;
  double max_abs_vel_deg_s = 0.0;
  if (hw_velocities_command_.size() >= 6)
  {
    for (size_t i = 0; i < 6; ++i)
    {
      double vel_rad_s = hw_velocities_command_[i];
      double abs_vel_deg_s = std::abs(vel_rad_s) * 180.0 / M_PI;
      if (abs_vel_deg_s > 1e-6)
      {
        has_velocity_command = true;
        // Use maximum absolute velocity as the common speed for all joints (matching Python SDK)
        max_abs_vel_deg_s = std::max(max_abs_vel_deg_s, abs_vel_deg_s);
      }
    }
  }
  
  // Use default speed if no velocity command provided, otherwise use maximum velocity
  double speed_deg_s = default_speed_rad_s_ * 180.0 / M_PI;
  if (has_velocity_command)
  {
    speed_deg_s = max_abs_vel_deg_s;
  }
  
  // Use unified set_joint_and_gripper method (matching Python SDK)
  // Joint and gripper are controlled in one frame with single speed for all joints
  data_parser_control_->set_joint_and_gripper(joint_angles, gripper_value, speed_deg_s);

  return return_type::OK;
}


}  // namespace alicia_d_driver

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(alicia_d_driver::AliciaDHardwareInterface, hardware_interface::SystemInterface)

