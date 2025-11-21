#include <gtest/gtest.h>
#include "../src/serial/SerialPort.h"

class ServerArduinoTest : public ::testing::Test {
protected:
    io_context io_ctx;
    SerialPort serial{io_ctx};
    
    void SetUp() override {
        ASSERT_TRUE(serial.open("/dev/ttyACM0", 115200));
    }
    
    void TearDown() override {
        serial.close();
    }
};

TEST_F(ServerArduinoTest, SendTrajectoryData) {
    serial.sendControlData(
        100.0, 105.0, 98.0,  // positions
        50.0, 50.0, 50.0,    // velocities
        0.5, 0.5, 0.5        // forces
    );
    
    // Should not throw exception
    SUCCEED();
}

TEST_F(ServerArduinoTest, ReceiveStatus) {
    serial.sendCommand("0");  // Stop command
    
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    if (serial.hasData()) {
        std::string msg = serial.readLine();
        EXPECT_NE(msg, "");
    }
}

TEST_F(ServerArduinoTest, SendThresholds) {
    serial.sendThresholds(20, 40);
    
    // Should complete without error
    SUCCEED();
}