import pytest
import numpy as np
from vision.AngleCalculator import AngleCalculator
from vision.FrameProcessor import FrameProcessor
from utils.DataBuffer import DataBuffer

class TestAngleCalculator:
    """Test angle calculation"""
    
    def test_angle_90_degrees(self):
        """Test known angle: vertical foot (90°)"""
        knee = (100, 200)
        ankle = (100, 300)  # Directly below
        foot = (200, 300)   # Horizontal from ankle
        
        result = AngleCalculator.calculate_angle(knee, ankle, foot)
        
        assert result.is_valid
        assert abs(result.angle_degrees - 90.0) < 1.0
    
    def test_angle_0_degrees(self):
        """Test straight line (0°)"""
        knee = (100, 200)
        ankle = (150, 250)
        foot = (200, 300)  # Same line
        
        result = AngleCalculator.calculate_angle(knee, ankle, foot)
        
        assert result.is_valid
        assert abs(result.angle_degrees - 0.0) < 1.0
    
    def test_angle_180_degrees(self):
        """Test opposite directions (180°)"""
        knee = (100, 100)
        ankle = (100, 200)
        foot = (100, 300)  # Opposite direction to knee
        
        result = AngleCalculator.calculate_angle(knee, ankle, foot)
        
        assert result.is_valid
        assert abs(result.angle_degrees - 180.0) < 1.0
    
    def test_calibration_offset(self):
        """Test calibration offset application"""
        angle = 87.5
        offset = -35.0
        
        compensated = AngleCalculator.apply_calibration_offset(angle, offset)
        
        assert compensated == pytest.approx(52.5)
    
    def test_angle_constraint(self):
        """Test angle constraining"""
        angle = 200.0
        constrained = AngleCalculator.constrain_angle(angle, 0.0, 180.0)
        
        assert constrained == 180.0


class TestDataBuffer:
    """Test data smoothing buffer"""
    
    def test_buffer_average(self):
        """Test average calculation"""
        buf = DataBuffer(max_size=5)
        
        buf.add_multiple([89.0, 90.0, 91.0, 90.0, 89.5])
        
        avg = buf.get_average()
        assert avg == pytest.approx(89.9)
    
    def test_buffer_median(self):
        """Test median calculation"""
        buf = DataBuffer(max_size=5)
        
        buf.add_multiple([85.0, 90.0, 95.0, 90.0, 92.0])
        
        median = buf.get_median()
        assert median == pytest.approx(90.0)
    
    def test_buffer_full(self):
        """Test buffer size limit"""
        buf = DataBuffer(max_size=3)
        
        buf.add_multiple([1, 2, 3, 4, 5])
        
        assert buf.size() == 3
        assert buf.get_all() == [3, 4, 5]
    
    def test_exponential_smoothing(self):
        """Test exponential smoothing"""
        buf = DataBuffer(max_size=5)
        
        buf.add_multiple([10, 20, 30, 40, 50])
        
        smoothed = buf.exponential_smooth(alpha=0.5)
        
        assert 20 < smoothed < 40


# Run tests
# pytest vision/tests/test_vision.py -v