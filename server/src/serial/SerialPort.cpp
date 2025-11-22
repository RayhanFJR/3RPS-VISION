//==================================================================
// FILE: server/src/serial/SerialPort.cpp
// Modernized non-blocking async implementation
//==================================================================

#include "SerialPort.h"
#include <iostream>
#include <sstream>
#include <fcntl.h>
#include <unistd.h>

SerialPort::SerialPort(boost::asio::io_context& io_ctx)
    : io_context(io_ctx),
      serial(io_ctx),
      is_open(false)
{
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

        // === IMPORTANT: Set serial to native non-blocking ===
        int flags = fcntl(serial.native_handle(), F_GETFL, 0);
        fcntl(serial.native_handle(), F_SETFL, flags | O_NONBLOCK);

        is_open = true;

        std::cout << "[SerialPort] Opened " << port 
                  << " @ " << baud_rate << " baud" << std::endl;

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
            std::cout << "[SerialPort] Closed" << std::endl;
        }
        catch (...) {}
    }
    is_open = false;
}

bool SerialPort::isOpen() const {
    return is_open && serial.is_open();
}

//==================================================================
// WRITE
//==================================================================

bool SerialPort::sendCommand(const std::string& cmd) {
    if (!isOpen()) return false;

    try {
        std::string msg = cmd + "\n";
        boost::asio::async_write(
            serial,
            boost::asio::buffer(msg),
            [](const boost::system::error_code&, std::size_t){}
        );
        return true;
    }
    catch (const std::exception& e) {
        std::cerr << "[SerialPort] Write error: " << e.what() << std::endl;
        return false;
    }
}

bool SerialPort::sendControlData(float pos1, float pos2, float pos3,
                                 float vel1, float vel2, float vel3,
                                 float fc1, float fc2, float fc3)
{
    std::ostringstream oss;
    oss << "S"
        << pos1 << "," << pos2 << "," << pos3 << ","
        << vel1 << "," << vel2 << "," << vel3 << ","
        << fc1  << "," << fc2  << "," << fc3;

    return sendCommand(oss.str());
}

bool SerialPort::sendManualCommand(char direction) {
    switch (direction) {
        case '0': return sendCommand("0");
        case '1': return sendCommand("1");
        case '2': return sendCommand("2");
        default:
            std::cerr << "[SerialPort] Invalid manual command" << std::endl;
            return false;
    }
}

bool SerialPort::sendCalibrate() {
    return sendCommand("X");
}

bool SerialPort::sendEmergencyStop() {
    return sendCommand("E");
}

//==================================================================
// READ (Non-blocking polling)
//==================================================================

std::string SerialPort::readAvailable() {
    if (!isOpen())
        return "";

    try {
        char data[256];
        boost::system::error_code ec;

        size_t n = serial.read_some(boost::asio::buffer(data), ec);

        if (ec) {
            if (ec == boost::asio::error::would_block ||
                ec == boost::asio::error::try_again)
                return "";
            return "";
        }

        return std::string(data, n);
    }
    catch (...) {
        return "";
    }
}

std::string SerialPort::readLine() {
    std::string data = readAvailable();
    static std::string buffer;

    if (data.empty())
        return "";

    buffer += data;

    auto pos = buffer.find('\n');
    if (pos != std::string::npos) {
        std::string line = buffer.substr(0, pos);
        buffer.erase(0, pos + 1);

        if (!line.empty() && line.back() == '\r')
            line.pop_back();

        return line;
    }

    return "";
}
