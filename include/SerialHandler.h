#ifndef SERIALHANDLER_H
#define SERIALHANDLER_H

#include <boost/asio.hpp>
#include <string>

using namespace boost::asio;

class SerialHandler {
public:
    SerialHandler(io_context& io);
    ~SerialHandler();
    
    // Initialization
    bool initialize(const std::string& port = "/dev/ttyACM0", int baudRate = 115200);
    void close();
    
    // Communication
    void sendCommand(const std::string& cmd);
    bool hasData();
    std::string readData();
    
    // Helper functions
    float parseValue(const std::string& data, const std::string& key);

private:
    serial_port serial;
    io_context& io;
    bool isOpen;
};

#endif // SERIALHANDLER_H

