#ifndef STATEHANDLERS_H
#define STATEHANDLERS_H

#include "StateMachine.h"
#include <modbus/modbus.h>
#include <boost/asio.hpp>
#include <chrono>
#include <string>

using namespace boost::asio;

// Forward declarations
class ControlHandler;
class ModbusHandler;
class SerialHandler;

SystemState handleIdleState(ModbusHandler& modbus, SerialHandler& serial,
                           ControlHandler& control, bool& animasi_grafik,
                           int& t_controller, int& t_grafik,
                           std::chrono::steady_clock::time_point& lastTraTime,
                           std::chrono::steady_clock::time_point& lastGrafikTime,
                           int& last_thresh1, int& last_thresh2);

SystemState handleResettingState(ModbusHandler& modbus);

SystemState handleAutoRetreatState(ModbusHandler& modbus, SerialHandler& serial,
                                   ControlHandler& control,
                                   std::string& arduinoFeedbackState, 
                                   bool& messageSend,
                                   std::chrono::steady_clock::time_point& lastTraTime);

SystemState handlePostRehabDelay(std::chrono::steady_clock::time_point& delayStartTime,
                                SerialHandler& serial, ModbusHandler& modbus,
                                ControlHandler& control,
                                bool& animasi_grafik, int& t_controller, int& t_grafik,
                                std::chrono::steady_clock::time_point& lastTraTime,
                                std::chrono::steady_clock::time_point& lastGrafikTime);

#endif // STATEHANDLERS_H

