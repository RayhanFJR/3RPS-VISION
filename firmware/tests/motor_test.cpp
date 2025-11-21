#include <gtest/gtest.h>
#include "../src/control/MotorController.h"

class MotorControllerTest : public ::testing::Test {
protected:
    MotorController motor;
    
    void SetUp() override {
        // Initialize for testing
    }
};

TEST_F(MotorControllerTest, ConstrainPWM) {
    // Test PWM constraining
    EXPECT_EQ(motor.constrainPWM(255), 255);
    EXPECT_EQ(motor.constrainPWM(500), 255);
    EXPECT_EQ(motor.constrainPWM(-300), -255);
}

TEST_F(MotorControllerTest, GainSetting) {
    // Test setting gains
    motor.setOuterLoopGains(1, 120.0, 0.15);
    
    EXPECT_FLOAT_EQ(motor.getKp(1), 120.0);
    EXPECT_FLOAT_EQ(motor.getKd(1), 0.15);
}

TEST_F(MotorControllerTest, LoadScaling) {
    // Test load-based scaling
    motor.setLoadScaling(15.0);  // Light load
    EXPECT_FLOAT_EQ(motor.getLoadScaling(), 1.0);
    
    motor.setLoadScaling(45.0);  // Heavy load
    EXPECT_FLOAT_EQ(motor.getLoadScaling(), 0.15);
}

// Compile: g++ -std=c++17 test_motor.cpp -lgtest -o test_motor
// Run: ./test_motor