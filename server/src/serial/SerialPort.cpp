//==================================================================
// FILE: server/src/serial/SerialPort.cpp
// UPDATED: Added non-blocking read functionality
//==================================================================

#include "SerialPort.h"
#include <iostream>
#include <sstream>

SerialPort::SerialPort(boost::asio::io_context& io_ctx)
    : io_context(io_ctx),
      serial(io_ctx),
      is_open(false) {
}

SerialPort::~SerialPort() {
    close();
}

bool SerialPort::open(const std::string& port, unsigned int baud_rate) {
    try {
        serial.open(port);
        serial.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
        serial.set_option(boost::asio::serial_port_base::character_size(8));
        serial.set_option(boost::asio::serial_port_base::parity(
            boost::asio::serial_port_base::parity::none));
        serial.set_option(boost::asio::serial_port_base::stop_bits(
            boost::asio::serial_port_base::stop_bits::one));
        
        is_open = true;
        
        std::cout << "[SerialPort] Opened " << port 
                  << " at " << baud_rate << " baud" << std::endl;
        
        return true;
    }
    catch (const std::exception& e) {
        std::cerr << "[SerialPort] Failed to open: " << e.what() << std::endl;
        is_open = false;
        return false;
    }
}

void SerialPort::close() {
    if (is_open && serial.is_open()) {
        try {
            serial.close();
            is_open = false;
            std::cout << "[SerialPort] Closed" << std::endl;
        }
        catch (const std::exception& e) {
            std::cerr << "[SerialPort] Error closing: " << e.what() << std::endl;
        }
    }
}

bool SerialPort::isOpen() const {
    return is_open && serial.is_open();
}

// ========== WRITE FUNCTIONS ==========

bool SerialPort::sendCommand(const std::string& cmd) {
    if (!isOpen()) {
        std::cerr << "[SerialPort] Cannot send - port closed" << std::endl;
        return false;
    }
    
    try {
        std::string full_cmd = cmd + "\n";
        boost::asio::write(serial, boost::asio::buffer(full_cmd));
        return true;
    }
    catch (const std::exception& e) {
        std::cerr << "[SerialPort] Write error: " << e.what() << std::endl;
        return false;
    }
}

bool SerialPort::sendControlData(float pos1, float pos2, float pos3,
                                 float vel1, float vel2, float vel3,
                                 float fc1, float fc2, float fc3) {
    if (!isOpen()) return false;
    
    try {
        std::ostringstream oss;
        oss << "C," 
            << pos1 << "," << pos2 << "," << pos3 << ","
            << vel1 << "," << vel2 << "," << vel3 << ","
            << fc1 << "," << fc2 << "," << fc3 << "\n";
        
        std::string data = oss.str();
        boost::asio::write(serial, boost::asio::buffer(data));
        return true;
    }
    catch (const std::exception& e) {
        std::cerr << "[SerialPort] Control write error: " << e.what() << std::endl;
        return false;
    }
}

bool SerialPort::sendManualCommand(char direction) {
    if (!isOpen()) return false;
    
    std::string cmd;
    switch (direction) {
        case '0': cmd = "M0\n"; break;  // Stop
        case '1': cmd = "M1\n"; break;  // Forward
        case '2': cmd = "M2\n"; break;  // Backward
        default:
            std::cerr << "[SerialPort] Invalid manual command: " << direction << std::endl;
            return false;
    }
    
    return sendCommand(cmd);
}

bool SerialPort::sendCalibrate() {
    return sendCommand("CAL");
}

bool SerialPort::sendEmergencyStop() {
    return sendCommand("ESTOP");
}

// ========== READ FUNCTIONS (NEW!) ==========

std::string SerialPort::readLine() {
    if (!isOpen()) {
        return "";
    }
    
    try {
        // Set non-blocking mode
        serial.non_blocking(true);
        
        // Read until newline with timeout
        boost::asio::streambuf buffer;
        boost::system::error_code ec;
        
        size_t n = boost::asio::read_until(serial, buffer, '\n', ec);
        
        if (ec) {
            // No data available or timeout - not an error
            if (ec == boost::asio::error::would_block || 
                ec == boost::asio::error::try_again ||
                ec == boost::asio::error::operation_aborted) {
                return "";
            }
            // Real error
            std::cerr << "[SerialPort] Read error: " << ec.message() << std::endl;
            return "";
        }
        
        if (n > 0) {
            std::istream is(&buffer);
            std::string line;
            std::getline(is, line);
            
            // Remove carriage return if present
            if (!line.empty() && line.back() == '\r') {
                line.pop_back();
            }
            
            return line;
        }
    }
    catch (const std::exception& e) {
        std::cerr << "[SerialPort] Read error: " << e.what() << std::endl;
    }
    
    return "";
}

std::string SerialPort::readAvailable() {
    if (!isOpen()) {
        return "";
    }
    
    try {
        if (serial.available() == 0) {
            return "";
        }
        
        // Read all available data
        boost::asio::streambuf buffer;
        size_t n = boost::asio::read(serial, buffer, 
                                    boost::asio::transfer_at_least(1));
        
        if (n > 0) {
            std::istream is(&buffer);
            std::string data;
            std::getline(is, data, '\0');  // Read until null or end
            return data;
        }
    }
    catch (const boost::system::system_error& e) {
        if (e.code() != boost::asio::error::would_block &&
            e.code() != boost::asio::error::try_again) {
            std::cerr << "[SerialPort] Read error: " << e.what() << std::endl;
        }
    }
    catch (const std::exception& e) {
        std::cerr << "[SerialPort] Read error: " << e.what() << std::endl;
    }
    
    return "";
}