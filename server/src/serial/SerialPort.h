//==================================================================
// FILE: server/src/serial/SerialPort.h
// UPDATED: Added read functionality
//==================================================================

#ifndef SERIAL_PORT_H
#define SERIAL_PORT_H

#include <boost/asio.hpp>
#include <string>

class SerialPort {
public:
    explicit SerialPort(boost::asio::io_context& io_ctx);
    ~SerialPort();
    
    // Connection management
    bool open(const std::string& port, unsigned int baud_rate);
    void close();
    bool isOpen() const;
    
    // Write functions
    bool sendCommand(const std::string& cmd);
    bool sendControlData(float pos1, float pos2, float pos3,
                        float vel1, float vel2, float vel3,
                        float fc1, float fc2, float fc3);
    bool sendManualCommand(char direction);  // '0'=stop, '1'=fwd, '2'=back
    bool sendCalibrate();
    bool sendEmergencyStop();
    
    // ========== NEW: Read functions ==========
    std::string readLine();        // Read until newline (non-blocking)
    std::string readAvailable();   // Read all available data (non-blocking)

private:
    boost::asio::io_context& io_context;
    boost::asio::serial_port serial;
    bool is_open;
};

#endif // SERIAL_PORT_H