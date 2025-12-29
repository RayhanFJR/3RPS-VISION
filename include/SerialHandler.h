#ifndef SERIALHANDLER_H
#define SERIALHANDLER_H

#include <boost/asio.hpp>
#include <string>

using namespace boost::asio;

class SerialHandler {
private:
    serial_port serial;
    io_context& io;
    bool isOpen;
    bool trajectoryPaused;  // NEW: Track pause state

public:
    SerialHandler(io_context& io);
    ~SerialHandler();
    
    bool initialize(const std::string& port, int baudRate);
    void close();
    void sendCommand(const std::string& cmd);
    bool hasData();
    std::string readData();
    float parseValue(const std::string& data, const std::string& key);
    
    // NEW METHODS for pause/resume handling
    void processArduinoFeedback(const std::string& data);
    bool isTrajectoryPaused() const { return trajectoryPaused; }
    void resetPauseState() { trajectoryPaused = false; }
};

#endif // SERIALHANDLER_H