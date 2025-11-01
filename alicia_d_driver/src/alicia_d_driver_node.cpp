#include "alicia_d_driver/alicia_d_driver_node.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <cmath>
#include <numeric> // For std::accumulate

constexpr uint8_t CMD_SERVO_CONTROL = 0x04;
constexpr uint8_t CMD_GRIPPER_CONTROL = 0x02;
constexpr uint8_t CMD_ZERO_CAL = 0x03;
constexpr uint8_t CMD_DEMO_CONTROL = 0x13;
constexpr uint8_t CMD_VERSION_QUERY = 0x0A;
// Protocol Constants for feedback frames
constexpr uint8_t FEEDBACK_GRIPPER_STATE = 0x02; // v5
constexpr uint8_t FEEDBACK_SERVO_STATE = 0x04;   // v5
constexpr uint8_t FEEDBACK_GRIPPER_STATE_V6 = 0x12;
constexpr uint8_t FEEDBACK_SERVO_STATE_V6 = 0x14;
constexpr uint8_t FEEDBACK_SERVO_STATE_EXT = 0x06;
constexpr uint8_t FEEDBACK_ERROR = 0xEE;
constexpr uint8_t FEEDBACK_VERSION = 0x0A;


AliciaDDriverNode::AliciaDDriverNode() : Node("alicia_d_driver_node"), last_process_time_(0, 0, RCL_ROS_TIME)
{
    declare_parameters();
    setup_ros_communications();

    // Attempt initial connection
    if (communicator_->connect()) {
        RCLCPP_INFO(this->get_logger(), "Initial connection successful. Enabling full torque mode.");

        auto frame = generate_simple_frame(CMD_DEMO_CONTROL, 0x01, true);
        communicator_->write_raw_frame(frame);
        // Firmware handling
        if (firmware_version_.empty() || firmware_version_ == "auto") {
            RCLCPP_INFO(this->get_logger(), "Firmware version not specified, attempting auto-detection...");
            send_firmware_query();
        } else {
            firmware_version_detected_ = true;
            firmware_new_ = (!firmware_version_.empty() && firmware_version_[0] >= '6');
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "Initial connection failed. Starting reconnect timer.");
        reconnect_timer_ = this->create_wall_timer(
            std::chrono::seconds(5),
            [this]() {
                if (!communicator_->is_connected()) {
                    RCLCPP_INFO(this->get_logger(), "Attempting to reconnect...");
                    if (communicator_->connect()) {
                        RCLCPP_INFO(this->get_logger(), "Reconnect successful! Enabling full torque mode.");
                        reconnect_timer_->cancel();
                        // Also send command after a successful reconnect
                        auto frame = generate_simple_frame(CMD_DEMO_CONTROL, 0x01, true);
                        communicator_->write_raw_frame(frame);
                    }
                } else {
                    reconnect_timer_->cancel();
                }
            }
        );
    }
}

AliciaDDriverNode::AliciaDDriverNode(const rclcpp::NodeOptions& options)
    : Node("alicia_d_driver_node", options), last_process_time_(0, 0, RCL_ROS_TIME)
{
    declare_parameters();
    setup_ros_communications();

    // Attempt initial connection
    if (communicator_->connect()) {
        RCLCPP_INFO(this->get_logger(), "Initial connection successful. Enabling full torque mode.");

        auto frame = generate_simple_frame(CMD_DEMO_CONTROL, 0x01, true);
        communicator_->write_raw_frame(frame);
        if (firmware_version_.empty() || firmware_version_ == "auto") {
            RCLCPP_INFO(this->get_logger(), "Firmware version not specified, attempting auto-detection...");
            send_firmware_query();
        } else {
            firmware_version_detected_ = true;
            firmware_new_ = (!firmware_version_.empty() && firmware_version_[0] >= '6');
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "Initial connection failed. Starting reconnect timer.");
        reconnect_timer_ = this->create_wall_timer(
            std::chrono::seconds(5),
            [this]() {
                if (!communicator_->is_connected()) {
                    RCLCPP_INFO(this->get_logger(), "Attempting to reconnect...");
                    if (communicator_->connect()) {
                        RCLCPP_INFO(this->get_logger(), "Reconnect successful! Enabling full torque mode.");
                        reconnect_timer_->cancel();
                        // Also send command after a successful reconnect
                        auto frame = generate_simple_frame(CMD_DEMO_CONTROL, 0x01, true);
                        communicator_->write_raw_frame(frame);
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
    if (communicator_) {
        communicator_->disconnect();
    }
}

void AliciaDDriverNode::declare_parameters()
{
    if (!this->has_parameter("port")) {
        this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
    }
    if (!this->has_parameter("baud_rate")) {
        this->declare_parameter<int>("baud_rate", 921600);
    }
    if (!this->has_parameter("debug_mode")) {
        this->declare_parameter<bool>("debug_mode", false);
    }
    if (!this->has_parameter("servo_count")) {
        this->declare_parameter<int>("servo_count", 9);
    }
    if (!this->has_parameter("rate_limit_sec")) {
        this->declare_parameter<double>("rate_limit_sec", 0.01);
    }
    // ROS1-like parameters (guarded to avoid double-declare with auto overrides)
    if (!this->has_parameter("command_rate_hz")) {
        this->declare_parameter<double>("command_rate_hz", 200.0);
    }
    if (!this->has_parameter("firmware_version")) {
        this->declare_parameter<std::string>("firmware_version", std::string("auto"));
    }
    if (!this->has_parameter("gripper_type")) {
        this->declare_parameter<std::string>("gripper_type", std::string("50mm"));
    }
    if (!this->has_parameter("default_speed_rad_s")) {
        this->declare_parameter<double>("default_speed_rad_s", 0.349);
    }
    if (!this->has_parameter("use_trajectory_smoothing")) {
        this->declare_parameter<bool>("use_trajectory_smoothing", false);
    }
    if (!this->has_parameter("max_joint_velocity_rad_s")) {
        this->declare_parameter<double>("max_joint_velocity_rad_s", 5.0);
    }
    if (!this->has_parameter("max_joint_accel_rad_s2")) {
        this->declare_parameter<double>("max_joint_accel_rad_s2", 20.0);
    }
    if (!this->has_parameter("max_gripper_velocity_units_s")) {
        this->declare_parameter<double>("max_gripper_velocity_units_s", 100.0);
    }
    if (!this->has_parameter("max_gripper_accel_units_s2")) {
        this->declare_parameter<double>("max_gripper_accel_units_s2", 100.0);
    }

    std::string port = this->get_parameter("port").as_string();
    uint32_t baud_rate = this->get_parameter("baud_rate").as_int();
    debug_mode_ = this->get_parameter("debug_mode").as_bool();
    servo_count_ = this->get_parameter("servo_count").as_int();
    rate_limit_sec_ = this->get_parameter("rate_limit_sec").as_double(); // Get rate limit
    command_rate_hz_ = this->get_parameter("command_rate_hz").as_double();
    firmware_version_ = this->get_parameter("firmware_version").as_string();
    gripper_type_ = this->get_parameter("gripper_type").as_string();
    default_speed_rad_s_ = this->get_parameter("default_speed_rad_s").as_double();
    use_trajectory_smoothing_ = this->get_parameter("use_trajectory_smoothing").as_bool();
    max_joint_velocity_rad_s_ = this->get_parameter("max_joint_velocity_rad_s").as_double();
    max_joint_accel_rad_s2_ = this->get_parameter("max_joint_accel_rad_s2").as_double();
    max_gripper_velocity_units_s_ = this->get_parameter("max_gripper_velocity_units_s").as_double();
    max_gripper_accel_units_s2_ = this->get_parameter("max_gripper_accel_units_s2").as_double();

    communicator_ = std::make_unique<SerialCommunicator>(port, baud_rate, debug_mode_);
    RCLCPP_INFO(this->get_logger(), "Configured port: %s, baud: %u, debug: %s",
                port.c_str(), baud_rate, debug_mode_ ? "true" : "false");

    joint_to_servo_map_index_ = {0, 0, 1, 1, 2, 2, 3, 4, 5};
    joint_to_servo_map_direction_ = {1.0, 1.0, 1.0, -1.0, 1.0, -1.0, 1.0, 1.0, 1.0};
    servo_to_joint_map_index_ = {0, -1, 1, -1, 2, -1, 3, 4, 5}; // -1 means ignore
    servo_to_joint_map_direction_ = {1.0, 0, 1.0, 0, 1.0, 0, 1.0, 1.0, 1.0};
}

void AliciaDDriverNode::setup_ros_communications()
{
    // Publishers
    joint_state_pub_std_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

    array_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/servo_states_main", 10);
    joint_command_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_commands", 10, std::bind(&AliciaDDriverNode::joint_command_callback, this, std::placeholders::_1));
        
    zero_calib_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/zero_calibrate", 10, std::bind(&AliciaDDriverNode::zero_calibrate_callback, this, std::placeholders::_1));
    demo_mode_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/demonstration", 10, std::bind(&AliciaDDriverNode::demonstration_mode_callback, this, std::placeholders::_1));

    // Main processing timer
    processing_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10), // Process incoming data at ~100Hz
        std::bind(&AliciaDDriverNode::process_serial_data, this));
    // Heartbeat publisher to keep /joint_states fresh
    heartbeat_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20),
        std::bind(&AliciaDDriverNode::heartbeat_publish_callback, this));
    // Decoupled command sender at configured rate
    auto cmd_period_ms = static_cast<int>(1000.0 / std::max(1.0, command_rate_hz_));
    command_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(cmd_period_ms),
        std::bind(&AliciaDDriverNode::send_command_timer_callback, this));
}


void AliciaDDriverNode::joint_command_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    // Cache latest, send from timer
    std::map<std::string, double> joint_map;
    for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
        joint_map[msg->name[i]] = msg->position[i];
    }
    std::vector<std::string> hardware_joint_names = {"Joint1","Joint2","Joint3","Joint4","Joint5","Joint6"};
    std::vector<double> joint_angles;
    for (const auto& joint_name : hardware_joint_names) {
        auto it = joint_map.find(joint_name);
        joint_angles.push_back(it != joint_map.end() ? it->second : 0.0);
    }
    double gripper_value = 0.0;
    auto it_grip = joint_map.find("right_finger");
    if (it_grip != joint_map.end()) {
        const double stroke_m = (gripper_type_ == "100mm") ? 0.05 : 0.025;
        const double m = std::max(0.0, std::min(stroke_m, it_grip->second));
        gripper_value = 100.0 - ((stroke_m > 1e-6 ? m / stroke_m : 0.0) * 100.0);
    }
    {
        std::lock_guard<std::mutex> lock(latest_cmd_mutex_);
        latest_joint_angles_ = joint_angles;
        latest_gripper_value_ = gripper_value;
        has_latest_command_ = true;
    }
}



uint16_t AliciaDDriverNode::rad_to_hardware_value_grip(double angle_rad)
{
    double angle_deg = angle_rad * 180.0 / M_PI;
    angle_deg = std::max(0.0, std::min(100.0, angle_deg)); // Clamp to expected [0, 100] deg range
    // Linear map [0, 100] deg to [2048, 2900]
    int hardware_value = static_cast<int>(angle_deg * 8.52 + 2048.0);
    return std::max(2048, std::min(2900, hardware_value));
}

uint16_t AliciaDDriverNode::rad_to_hardware_value(double angle_rad) {
    double angle_deg = angle_rad * 180.0 / M_PI;
    angle_deg = std::max(-180.0, std::min(180.0, angle_deg));
    int value = static_cast<int>((angle_deg + 180.0) / 360.0 * 4096.0);
    return std::max(0, std::min(4095, value));
}


uint8_t AliciaDDriverNode::calculate_checksum(const std::vector<uint8_t>& frame_data)
{
    // The checksum is the sum of the DATA PAYLOAD bytes, modulo 2.
    // The frame_data vector is passed in *before* the checksum is calculated and inserted.
    // The payload starts at index 3 and its length is specified at index 2.
    if (frame_data.size() < 4) {
        return 0; // Frame is too short to have a payload
    }
    
    // The length of the actual data payload.
    const uint8_t payload_len = frame_data[2];

    // Ensure the frame is large enough to contain the declared payload
    if (frame_data.size() < (size_t)3 + payload_len) {
        return 0; 
    }

    // Sum from the beginning of the payload (index 3) for the length of the payload.
    int sum = std::accumulate(frame_data.begin() + 3, 
                              frame_data.begin() + 3 + payload_len, 
                              0);

    return static_cast<uint8_t>(sum % 2);
}

std::vector<uint8_t> AliciaDDriverNode::generate_simple_frame(uint8_t command, uint8_t data, bool use_checksum)
{
    std::vector<uint8_t> frame(6);
    frame[0] = FRAME_START_BYTE;
    frame[1] = command;
    frame[2] = 0x01; // Data length is always 1 for these simple frames
    frame[3] = data & 0xFF; // Data byte

    if (use_checksum) {
        // Checksum is just the data byte modulo 2, as per the Python logic
        frame[4] = data % 2;
    } else {
        frame[4] = 0x00;
    }

    frame[5] = FRAME_END_BYTE;
    return frame;
}

void AliciaDDriverNode::zero_calibrate_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (msg->data) {
        RCLCPP_INFO(this->get_logger(), "Received Zero Calibration command.");
        auto frame = generate_simple_frame(CMD_ZERO_CAL, 0x00, false);
        communicator_->write_raw_frame(frame);
    }
}

void AliciaDDriverNode::demonstration_mode_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (msg->data) {
        RCLCPP_INFO(this->get_logger(), "Enabling Demonstration Mode (Zero Torque).");
        auto frame = generate_simple_frame(CMD_DEMO_CONTROL, 0x00, false);
        communicator_->write_raw_frame(frame);
    } else {
        RCLCPP_INFO(this->get_logger(), "Disabling Demonstration Mode (Full Torque).");
        auto frame = generate_simple_frame(CMD_DEMO_CONTROL, 0x01, true);
        communicator_->write_raw_frame(frame);
    }

}


void AliciaDDriverNode::process_serial_data()
{
    if (!communicator_->is_connected()) return;

    std::vector<uint8_t> packet;
    // Process all available packets in the queue
    while (communicator_->get_packet(packet)) {
        if (packet.empty()) {
            RCLCPP_WARN(this->get_logger(), "Received an empty packet.");
            continue;
        }

        // Full frame format: AA CMD LEN DATA... CHK FF
        // packet[0] = AA (frame header)
        // packet[1] = CMD (command ID)
        // packet[2] = LEN (data length)
        // packet[3..2+LEN] = DATA (payload)
        // packet[3+LEN] = CHK (checksum)
        // packet[3+LEN+1] = FF (end byte)
        
        if (packet.size() < 4 || packet[0] != 0xAA) {
            RCLCPP_WARN(this->get_logger(), "Invalid frame: expected AA header, got size=%zu, first_byte=0x%02X", 
                       packet.size(), packet.size() > 0 ? packet[0] : 0);
            continue;
        }
        
        uint8_t command_id = packet[1];  // Command is at index 1 (after AA)
        
        // Extract data payload: includes LEN byte to match parse function expectations
        // Parse functions expect: LEN DATA...
        std::vector<uint8_t> data_payload;
        if (packet.size() >= 4) {
            uint8_t data_len = packet[2];
            if (packet.size() >= (size_t)3 + data_len + 2)  // AA CMD LEN DATA CHK FF
            {
                // Payload includes LEN byte: from index 2 (LEN) to index 3+LEN (before checksum)
                data_payload.assign(packet.begin() + 2, packet.begin() + 3 + data_len);
            }
        }

        switch (command_id) {
            case FEEDBACK_SERVO_STATE:
                parse_servo_states_frame(data_payload);
                break;
            case FEEDBACK_GRIPPER_STATE:
                parse_gripper_state_frame(data_payload);
                break;
            case FEEDBACK_SERVO_STATE_V6:
                parse_servo_states_frame(data_payload);
                break;
            case FEEDBACK_GRIPPER_STATE_V6:
                parse_gripper_state_frame(data_payload);
                break;
            case FEEDBACK_ERROR:
                parse_error_frame(data_payload);
                break;
            case FEEDBACK_VERSION:
                parse_version_frame(data_payload);
                break;
            case FEEDBACK_SERVO_STATE_EXT:
                if (debug_mode_) {
                    RCLCPP_INFO(this->get_logger(), "Received unhandled Extended Servo State frame (0x06)");
                }
                break;
            default:
                RCLCPP_WARN(this->get_logger(), "Received frame with unhandled command ID: 0x%02X", command_id);
                break;
        }
    }
}

void AliciaDDriverNode::parse_servo_states_frame(const std::vector<uint8_t>& data_payload)
{
    // Rate Limiting
    auto now = this->get_clock()->now();
    if ((now - last_process_time_).seconds() < rate_limit_sec_) {
        return; // Skip processing if too soon
    }
    last_process_time_ = now;

    // The data_payload contains the data bytes *after* the command ID.
    // The first byte of this payload is the data length.
    if (data_payload.empty()) {
        RCLCPP_WARN(this->get_logger(), "Servo state data payload is empty.");
        return;
    }
    uint8_t data_byte_count = data_payload[0];
    if (data_payload.size() < (size_t)data_byte_count + 1) {
        RCLCPP_WARN(this->get_logger(), "Servo state data payload is shorter than its own length field indicates.");
        return;
    }
    int servos_in_frame = data_byte_count / 2;

    // Data Processing & State Update
    std::vector<double> joint_values(6, 0.0);
    for (int i = 0; i < servos_in_frame && i < servo_count_; ++i) {
        size_t data_idx = 1 + i * 2; // Data starts after the length byte.
        if (data_idx + 1 >= data_payload.size()) break; // Bounds check
        uint16_t hw_val = data_payload[data_idx] | (data_payload[data_idx + 1] << 8);
        double rad_val = hardware_value_to_rad(hw_val);

        if (static_cast<size_t>(i) < servo_to_joint_map_index_.size()) {
            int joint_idx = servo_to_joint_map_index_[i];
            if (joint_idx != -1) {
                joint_values[joint_idx] = rad_val * servo_to_joint_map_direction_[i];
            }
        }
    }

    sensor_msgs::msg::JointState js_msg;
    js_msg.header.stamp = now;
    js_msg.name = {"Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6"};
    js_msg.position = joint_values;
    joint_state_pub_std_->publish(js_msg);
    // update stored positions for heartbeat
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        current_joint_positions_ = joint_values;
    }
    

    // Publish Backward Compatibility Message
    std_msgs::msg::Float32MultiArray compat_msg;
    compat_msg.data = {(float)joint_values[0], (float)joint_values[1], (float)joint_values[2], (float)joint_values[3], (float)joint_values[4], (float)joint_values[5], (float)current_gripper_position_m_};
    array_pub_->publish(compat_msg);
}

void AliciaDDriverNode::parse_gripper_state_frame(const std::vector<uint8_t>& data_payload)
{
    // The data_payload is what comes *after* the command ID (0x02).
    // The structure is [LEN, ID, Low, High, ?, ?, BTN1, BTN2].
    // The warning log shows the payload size is 8 bytes.
    if (data_payload.size() < 8) {
        RCLCPP_WARN(this->get_logger(), "Gripper state data payload is smaller than the expected 8 bytes: %zu bytes", data_payload.size());
        return;
    }

    uint16_t gripper_hw_val = data_payload[2] | (data_payload[3] << 8);
    current_gripper_position_m_ = hardware_value_to_rad_grip(gripper_hw_val);
}


void AliciaDDriverNode::parse_error_frame(const std::vector<uint8_t>& payload)
{
    // Based on Python: error_type at index 3 -> 1 of command payload
    // error_param at index 4 -> 2 of command payload
    if (payload.size() < 2) {
        RCLCPP_WARN(this->get_logger(), "Error frame payload too short: %zu bytes", payload.size());
        return;
    }
    uint8_t error_type = payload[0];
    uint8_t error_param = payload[1];
    RCLCPP_ERROR(this->get_logger(), "Received Error Frame from Hardware: Type=0x%02X, Param=0x%02X", error_type, error_param);
}


double AliciaDDriverNode::hardware_value_to_rad(uint16_t hw_value) {
    hw_value = std::max(0, std::min(4095, (int)hw_value));
    double angle_deg = -180.0 + (static_cast<double>(hw_value) / 4095.0) * 360.0;
    return angle_deg * M_PI / 180.0;
}

double AliciaDDriverNode::hardware_value_to_rad_grip(uint16_t hw_value) {
    hw_value = std::max(2048, std::min(2900, (int)hw_value));
    // Linear map [2048, 2900] back to [0, 100] deg
    double angle_deg = (static_cast<double>(hw_value) - 2048.0) / 8.52;
    // Convert deg (0..100 open percent) to meters for prismatic publication
    double open_pct = std::max(0.0, std::min(100.0, angle_deg));
    const double stroke_m = (gripper_type_ == "100mm") ? 0.05 : 0.025;
    double meters = (1.0 - (open_pct / 100.0)) * stroke_m;
    return meters;
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
    auto now = this->get_clock()->now();
    sensor_msgs::msg::JointState js;
    js.header.stamp = now;
    js.name = joint_names_;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        js.position = current_joint_positions_;
        js.position.push_back(current_gripper_position_m_);
    }
    joint_state_pub_std_->publish(js);
}

void AliciaDDriverNode::send_command_timer_callback()
{
    if (!communicator_ || !communicator_->is_connected()) return;
    if (!has_latest_command_) return;

    std::vector<double> joint_angles;
    double gripper_value = 0.0; // 0..100
    {
        std::lock_guard<std::mutex> lock(latest_cmd_mutex_);
        joint_angles = latest_joint_angles_;
        gripper_value = latest_gripper_value_;
    }

    // Build and send servo frame
    size_t frame_size = servo_count_ * 2 + 5;
    std::vector<uint8_t> servo_frame(frame_size);
    servo_frame[0] = FRAME_START_BYTE;
    servo_frame[1] = CMD_SERVO_CONTROL;
    servo_frame[2] = servo_count_ * 2;
    for (int i = 0; i < servo_count_; ++i) {
        uint16_t hw_val = 2048;
        if (static_cast<size_t>(i) < joint_to_servo_map_index_.size()) {
            int joint_idx = joint_to_servo_map_index_[i];
            double direction = joint_to_servo_map_direction_[i];
            if (static_cast<size_t>(joint_idx) < joint_angles.size()) {
                hw_val = rad_to_hardware_value(joint_angles[joint_idx] * direction);
            }
        }
        size_t idx = 3 + i * 2;
        servo_frame[idx] = hw_val & 0xFF;
        servo_frame[idx + 1] = (hw_val >> 8) & 0xFF;
    }
    servo_frame[frame_size - 2] = calculate_checksum(servo_frame);
    servo_frame[frame_size - 1] = FRAME_END_BYTE;
    communicator_->write_raw_frame(servo_frame);

    // Build and send gripper frame
    if (firmware_new_) {
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
    } else {
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

void AliciaDDriverNode::send_firmware_query()
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

void AliciaDDriverNode::parse_version_frame(const std::vector<uint8_t>& data_payload)
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
    RCLCPP_INFO(this->get_logger(), "Firmware version detected: %s (new=%s)", firmware_version_.c_str(), firmware_new_ ? "true" : "false");
}