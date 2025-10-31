#ifndef SERIAL_COMMUNICATOR_HPP
#define SERIAL_COMMUNICATOR_HPP

#include <string>
#include <vector>
#include <memory>
#include <thread>
#include <mutex>
#include <deque>
#include <atomic>
#include <libserial/SerialPort.h>
#include "rclcpp/rclcpp.hpp"

// Define the protocol constants
constexpr uint8_t FRAME_START_BYTE = 0xAA;
constexpr uint8_t FRAME_END_BYTE = 0xFF;
constexpr size_t MAX_FRAME_LENGTH = 64;

class SerialCommunicator
{
public:
    // Constructor now takes configuration with default values
    explicit SerialCommunicator(
        std::string port_name = "/dev/ttyUSB0",
        uint32_t baud_rate = 921600,
        bool debug_mode = false,
        rclcpp::Logger logger = rclcpp::get_logger("SerialCommunicator"));

    ~SerialCommunicator();

    // Connect now uses the stored parameters
    bool connect();
    void disconnect();
    bool is_connected() const;

    bool write_packet(const std::vector<uint8_t>& payload);
    bool write_raw_frame(const std::vector<uint8_t>& frame);
    bool get_packet(std::vector<uint8_t>& buffer);

private:
    void read_thread_loop();
    uint8_t sumElements(const std::vector<uint8_t>& data) const; // Checksum calculation - VERIFY THIS LOGIC!
    bool validate_checksum(const std::vector<uint8_t>& frame) const;
    uint8_t calculate_checksum(const std::vector<uint8_t>& frame) const;
    void print_hex_frame(const std::string& prefix, const std::vector<uint8_t>& data) const;
    std::string current_port_path_; // Actual connected port (e.g., "/dev/ttyUSB0")

    // Configuration
    std::string port_name_;
    uint32_t baud_rate_;
    bool debug_mode_;

    // State
    // std::unique_ptr<drivers::serial_driver::SerialPort> serial_port_;
    LibSerial::SerialPort serial_port_;
    std::thread read_thread_;
    std::atomic<bool> is_running_;
    std::mutex queue_mutex_;
    std::deque<std::vector<uint8_t>> received_packets_queue_;
    rclcpp::Logger logger_; // Logger for debug messages
};

#endif // SERIAL_COMMUNICATOR_HPP