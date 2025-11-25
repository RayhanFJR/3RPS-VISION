#include "SerialHandler.h"
#include "Config.h"
#include <iostream>
#include <sys/ioctl.h>
#include <termios.h>
#include <array>

SerialHandler::SerialHandler(io_context& io) : serial(io), io(io), isOpen(false) {
}

SerialHandler::~SerialHandler() {
    if (isOpen) {
        close();
    }
}

bool SerialHandler::initialize(const std::string& port, int baudRate) {
    try {
        serial.open(port);
        serial.set_option(serial_port_base::baud_rate(baudRate));
        isOpen = true;
        return true;
    } catch (const boost::system::system_error& e) {
        std::cerr << "Error membuka port serial: " << e.what() << std::endl;
        isOpen = false;
        return false;
    }
}

void SerialHandler::close() {
    if (isOpen) {
        serial.close();
        isOpen = false;
    }
}

void SerialHandler::sendCommand(const std::string& cmd) {
    if (!isOpen) return;
    
    std::string message = cmd + "\n";
    try {
        serial.write_some(boost::asio::buffer(message));
    } catch (const boost::system::system_error& e) {
        std::cerr << "Error sending command: " << e.what() << std::endl;
    }
}

bool SerialHandler::hasData() {
    if (!isOpen) return false;
    
    int bytes_available = 0;
    ::ioctl(serial.native_handle(), FIONREAD, &bytes_available);
    return bytes_available > 0;
}

std::string SerialHandler::readData() {
    if (!isOpen || !hasData()) return "";
    
    std::array<char, 256> buf;
    boost::system::error_code error;
    size_t len = serial.read_some(boost::asio::buffer(buf), error);
    
    if (len > 0 && !error) {
        return std::string(buf.data(), len);
    }
    
    return "";
}

float SerialHandler::parseValue(const std::string& data, const std::string& key) {
    size_t key_pos = data.find(key);
    if (key_pos == std::string::npos) return -1.0f;

    size_t value_start = key_pos + key.length();
    size_t value_end = data.find_first_of(",\n\r", value_start);
    
    try {
        return std::stof(data.substr(value_start, value_end - value_start));
    } catch (const std::exception& e) {
        return -1.0f;
    }
}

