#include "serial_communicator.hpp"
#include <rclcpp/logging.hpp>
#include <chrono>
#include <numeric>
#include <iomanip>
#include <sstream>
#include <unistd.h>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>


static LibSerial::BaudRate int_to_baudrate(uint32_t baud) {
    switch (baud) {
        case 50: return LibSerial::BaudRate::BAUD_50;
        case 75: return LibSerial::BaudRate::BAUD_75;
        case 110: return LibSerial::BaudRate::BAUD_110;
        case 134: return LibSerial::BaudRate::BAUD_134;
        case 150: return LibSerial::BaudRate::BAUD_150;
        case 200: return LibSerial::BaudRate::BAUD_200;
        case 300: return LibSerial::BaudRate::BAUD_300;
        case 600: return LibSerial::BaudRate::BAUD_600;
        case 1200: return LibSerial::BaudRate::BAUD_1200;
        case 1800: return LibSerial::BaudRate::BAUD_1800;
        case 2400: return LibSerial::BaudRate::BAUD_2400;
        case 4800: return LibSerial::BaudRate::BAUD_4800;
        case 9600: return LibSerial::BaudRate::BAUD_9600;
        case 19200: return LibSerial::BaudRate::BAUD_19200;
        case 38400: return LibSerial::BaudRate::BAUD_38400;
        case 57600: return LibSerial::BaudRate::BAUD_57600;
        case 115200: return LibSerial::BaudRate::BAUD_115200;
        case 230400: return LibSerial::BaudRate::BAUD_230400;
        case 460800: return LibSerial::BaudRate::BAUD_460800;
        case 500000: return LibSerial::BaudRate::BAUD_500000;
        case 576000: return LibSerial::BaudRate::BAUD_576000;
        case 921600: return LibSerial::BaudRate::BAUD_921600;
        case 1000000: return LibSerial::BaudRate::BAUD_1000000;
        case 1152000: return LibSerial::BaudRate::BAUD_1152000;
        case 1500000: return LibSerial::BaudRate::BAUD_1500000;
        case 2000000: return LibSerial::BaudRate::BAUD_2000000;
        case 2500000: return LibSerial::BaudRate::BAUD_2500000;
        case 3000000: return LibSerial::BaudRate::BAUD_3000000;
        case 3500000: return LibSerial::BaudRate::BAUD_3500000;
        case 4000000: return LibSerial::BaudRate::BAUD_4000000;
        default:
            throw std::invalid_argument("Unsupported baud rate integer value.");
    }
}



SerialCommunicator::SerialCommunicator(std::string port_name, uint32_t baud_rate, bool debug_mode, rclcpp::Logger logger)
    : port_name_(std::move(port_name)),
      baud_rate_(baud_rate),
      debug_mode_(debug_mode),
      is_running_(false),
      logger_(logger)
{
    RCLCPP_INFO(logger_, "SerialCommunicator created for port '%s' at %u bps.", port_name_.c_str(), baud_rate_);    
    if (debug_mode_) {
        RCLCPP_INFO(logger_, "Debug mode is enabled.");
    }
}

SerialCommunicator::~SerialCommunicator()
{
    disconnect();
}

bool SerialCommunicator::connect()
{
    if (is_connected()) {
        return true;
    }
    RCLCPP_INFO(logger_, "Attempting to open serial port: %s @ %u bps", port_name_.c_str(), baud_rate_);

    try {
        serial_port_.Open(port_name_);
        LibSerial::BaudRate baud_enum = int_to_baudrate(baud_rate_);
        serial_port_.SetBaudRate(baud_enum);
        serial_port_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
        serial_port_.SetParity(LibSerial::Parity::PARITY_NONE);
        serial_port_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
        serial_port_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
        
        serial_port_.SetDTR(true);
        serial_port_.SetRTS(true);

    } catch (const LibSerial::AlreadyOpen& e) {
        RCLCPP_WARN(logger_, "Serial port was already open: %s", e.what());
    } catch (const LibSerial::OpenFailed& e) {
        RCLCPP_ERROR(logger_, "Failed to open serial port %s: %s", port_name_.c_str(), e.what());
        return false;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "An unexpected error occurred while opening port: %s", e.what());
        return false;
    }
    is_running_ = true;
    read_thread_ = std::thread(&SerialCommunicator::read_thread_loop, this);
    RCLCPP_INFO(logger_, "Serial port %s opened successfully. Read thread started.", port_name_.c_str());
    return true;
}

void SerialCommunicator::disconnect()
{
    is_running_ = false;
    if (read_thread_.joinable()) {
        read_thread_.join();
    }
    if (serial_port_.IsOpen()) {
        try {
            serial_port_.Close();
            RCLCPP_INFO(logger_, "Serial port disconnected.");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(logger_, "Exception while closing port: %s", e.what());
        }
    }
}

bool SerialCommunicator::is_connected() const
{
    return serial_port_.IsOpen();
}

bool SerialCommunicator::get_packet(std::vector<uint8_t>& buffer)
{
    std::lock_guard<std::mutex> lock(queue_mutex_);
    if (received_packets_queue_.empty()) {
        return false;
    }
    buffer = received_packets_queue_.front();
    received_packets_queue_.pop_front();
    return true;
}

bool SerialCommunicator::write_packet(const std::vector<uint8_t>& payload)
{
    if (!is_connected()) {
        RCLCPP_WARN(logger_, "Write attempt failed: port is not open.");
        return false;
    }
    if (payload.size() > 255) {
        RCLCPP_ERROR(logger_, "Payload size %zu exceeds maximum of 255.", payload.size());
        return false;
    }

    std::vector<uint8_t> frame_to_send;
    frame_to_send.push_back(FRAME_START_BYTE);
    frame_to_send.push_back(static_cast<uint8_t>(payload.size()));
    frame_to_send.insert(frame_to_send.end(), payload.begin(), payload.end());
    
    uint8_t checksum = calculate_checksum(frame_to_send);
    frame_to_send.push_back(checksum);
    frame_to_send.push_back(FRAME_END_BYTE);

    print_hex_frame("Sending: ", frame_to_send);

    try {
        serial_port_.Write(frame_to_send);
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "Exception while writing to serial port: %s. Disconnecting.", e.what());
        disconnect();
        return false;
    }
}



bool SerialCommunicator::write_raw_frame(const std::vector<uint8_t>& frame)
{
    if (!is_connected()) {
        RCLCPP_WARN(logger_, "Write raw frame failed: port is not open.");
        return false;
    }
    // print_hex_frame("Sending Raw: ", frame);
    std::lock_guard<std::mutex> lock(queue_mutex_);
    try {
        serial_port_.Write(frame);
        try {
             serial_port_.FlushOutputBuffer();
             if (debug_mode_) {
                 RCLCPP_DEBUG(logger_, "Output buffer flushed for port %s.", port_name_.c_str());
             }
        } catch (const std::exception& flush_e) {
            RCLCPP_WARN(logger_, "Exception during flushing output buffer for port %s: %s",
                        port_name_.c_str(), flush_e.what());
        }

    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "Exception while writing raw frame: %s. Disconnecting.", e.what());
        disconnect();
        return false;
    }
    return true;
}

void SerialCommunicator::read_thread_loop()
{
    std::vector<uint8_t> frame_buffer;
    bool wait_for_start = true;
    const size_t MAX_FRAME_LENGTH = 64; // Safety limit

    RCLCPP_INFO(logger_, "Read thread started (Protocol: AA...FF).");

    while (is_running_) {
        if (!is_connected()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            continue;
        }

        uint8_t byte_buffer;
        try {
            // Read one byte with a short timeout
            serial_port_.ReadByte(byte_buffer, 100); // 100ms timeout

            if (wait_for_start) {
                if (byte_buffer == 0xAA) { // FRAME_START_BYTE
                    frame_buffer.clear();
                    frame_buffer.push_back(byte_buffer);
                    wait_for_start = false;
                }
            } else { // Already in a frame
                frame_buffer.push_back(byte_buffer);

                // Frame must have at least 5 bytes to be valid (AA CMD LEN CHK FF)
                if (frame_buffer.size() >= 5) {

                    // Check for end of frame
                    if (byte_buffer == 0xFF) { // FRAME_END_BYTE
                        uint8_t payload_len = frame_buffer[2];
                        size_t expected_total_len = (size_t)payload_len + 5;

                        if (frame_buffer.size() == expected_total_len) {
                            if (validate_checksum(frame_buffer)) {
                                std::lock_guard<std::mutex> lock(queue_mutex_);
                                received_packets_queue_.push_back(
                                    std::vector<uint8_t>(frame_buffer.begin() + 1, frame_buffer.end() - 2)
                                );
                                // print_hex_frame("Recv OK: ", frame_buffer);
                            } else {
                                print_hex_frame("Recv BAD CHECKSUM: ", frame_buffer);
                            }
                            wait_for_start = true; // Reset for the next frame
                        } else if (frame_buffer.size() > expected_total_len) {
                             print_hex_frame("Recv BAD LENGTH: ", frame_buffer);
                             wait_for_start = true; // Reset
                        }
                        // Reset for the next frame
                        wait_for_start = true;
                    }
                }
                
                // Safety break for corrupted frames
                if (frame_buffer.size() >= MAX_FRAME_LENGTH) {
                    print_hex_frame("Recv OVERFLOW/CORRUPT: ", frame_buffer);
                    wait_for_start = true;
                }
            }
        } catch (const LibSerial::ReadTimeout&) {
            // This is normal, just continue the loop
            continue;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(logger_, "Exception in read thread: %s. Disconnecting.", e.what());
            disconnect();
            wait_for_start = true; // Reset state on disconnect
        }
    }
    RCLCPP_INFO(logger_, "Read thread finished.");
}



// This function validates a complete frame (AA...FF)
bool SerialCommunicator::validate_checksum(const std::vector<uint8_t>& frame) const
{
    if (frame.size() < 4) return false;

    uint8_t received_checksum = sumElements(frame);
    uint8_t calculated_checksum = frame[frame.size() - 2]; // Checksum is the second last byte

    return received_checksum == calculated_checksum;
}


uint8_t SerialCommunicator::sumElements(const std::vector<uint8_t>& data) const
{
    // ROS 1 used >= 4.
     if (data.size() < 4) {
        // Log using ROS 2 logger
        RCLCPP_ERROR(logger_, "Cannot calculate ROS 1 checksum: Data size too small (%zu < 4).", data.size());
        return 0; // Return a predictable value on error
     }

     uint32_t sum = 0;
     // Sum from index 3 (typically first data byte after LEN) up to (but not including) checksum byte
     for (size_t i = 3; i < data.size() - 2; ++i) {
        sum += data[i];
     }

     // Return sum modulo 2 (parity)
     return sum % 2;
}



// This function calculates checksum on a complete frame (AA...FF)
uint8_t SerialCommunicator::calculate_checksum(const std::vector<uint8_t>& frame_data) const
{
    // The checksum is the sum of the PAYLOAD bytes, modulo 2.
    // In a full AA...FF frame, the payload starts at index 3.
    if (frame_data.size() < 5) {
        return 0; // Cannot calculate on an invalid frame
    }
    
    // Sum from the beginning of the payload (index 3) up to (but not including) the checksum byte.
    int sum = std::accumulate(frame_data.begin() + 3, frame_data.end() - 2, 0);
    
    return static_cast<uint8_t>(sum % 2);
}




void SerialCommunicator::print_hex_frame(const std::string& prefix, const std::vector<uint8_t>& data) const
{
    if (!debug_mode_) {
        return;
    }
    std::stringstream ss;
    ss << prefix << "[" << data.size() << " bytes]: ";
    ss << std::hex << std::uppercase << std::setfill('0');
    for (const auto& byte : data) {
        ss << std::setw(2) << static_cast<int>(byte) << " ";
    }
    RCLCPP_INFO(logger_, "%s", ss.str().c_str());
}



