import serial
import time
import pytest

class TestArduinoSerial:
    """Test Arduino serial communication"""
    
    @pytest.fixture
    def arduino(self):
        """Connect to Arduino"""
        ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        time.sleep(2)  # Wait for Arduino to initialize
        yield ser
        ser.close()
    
    def test_arduino_responds(self, arduino):
        """Test Arduino responds to commands"""
        arduino.write(b'0\n')  # Stop command
        time.sleep(0.2)
        
        # Arduino should respond
        response = arduino.readline().decode().strip()
        assert response != ""
    
    def test_manual_forward(self, arduino):
        """Test manual forward command"""
        arduino.write(b'1\n')
        time.sleep(0.5)
        
        # Check status
        arduino.write(b'X\n')  # Get status
        time.sleep(0.1)
        response = arduino.readline().decode()
        
        assert "status:" in response or "ready" in response.lower()
    
    def test_trajectory_command(self, arduino):
        """Test trajectory command format"""
        cmd = b'S100,110,120,50,51,52,0.5,0.5,0.5\n'
        arduino.write(cmd)
        time.sleep(0.5)
        
        # Should execute without error
        response = arduino.readline().decode()
        assert "error" not in response.lower()

# Run: pytest firmware/tests/test_serial_integration.py -v