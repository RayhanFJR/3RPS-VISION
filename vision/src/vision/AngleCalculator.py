#==================================================================
# FILE 2: vision/src/vision/AngleCalculator.py
#==================================================================

import numpy as np
from dataclasses import dataclass
from typing import Optional, Tuple

@dataclass
class AngleResult:
    """Result of angle calculation"""
    angle_degrees: float
    raw_angle: float
    is_valid: bool
    error_message: Optional[str] = None


class AngleCalculator:
    """
    Calculate foot angle from 3 body landmarks
    Angle: knee → ankle → foot_index
    """
    
    @staticmethod
    def calculate_angle(point_a: Tuple[float, float],
                       point_b: Tuple[float, float],
                       point_c: Tuple[float, float]) -> AngleResult:
        """
        Calculate angle from 3 points using vectors
        
        Points:
            A (knee) ------- B (ankle) ------- C (foot)
        
        Angle calculated at B (ankle)
        
        Args:
            point_a: Knee position (x, y)
            point_b: Ankle position (x, y) - vertex
            point_c: Foot position (x, y)
            
        Returns:
            AngleResult with angle in degrees
        """
        try:
            # Convert to numpy arrays
            a = np.array(point_a, dtype=np.float32)
            b = np.array(point_b, dtype=np.float32)
            c = np.array(point_c, dtype=np.float32)
            
            # Create vectors
            ba = a - b  # Vector from ankle to knee
            bc = c - b  # Vector from ankle to foot
            
            # Calculate angle using dot product
            # cos(angle) = (ba · bc) / (|ba| * |bc|)
            dot_product = np.dot(ba, bc)
            magnitude_ba = np.linalg.norm(ba)
            magnitude_bc = np.linalg.norm(bc)
            
            # Check for zero magnitude
            if magnitude_ba < 1e-6 or magnitude_bc < 1e-6:
                return AngleResult(
                    angle_degrees=0.0,
                    raw_angle=0.0,
                    is_valid=False,
                    error_message="Invalid point positions (zero magnitude)"
                )
            
            # Clamp cosine to [-1, 1] to avoid numerical errors
            cosine_angle = np.clip(
                dot_product / (magnitude_ba * magnitude_bc),
                -1.0, 1.0
            )
            
            # Calculate angle in radians and convert to degrees
            angle_radians = np.arccos(cosine_angle)
            angle_degrees = np.degrees(angle_radians)
            
            return AngleResult(
                angle_degrees=angle_degrees,
                raw_angle=angle_degrees,
                is_valid=True
            )
        
        except Exception as e:
            return AngleResult(
                angle_degrees=0.0,
                raw_angle=0.0,
                is_valid=False,
                error_message=str(e)
            )
    
    @staticmethod
    def calculate_angle_from_landmarks(landmarks) -> AngleResult:
        """
        Calculate angle directly from BodyLandmarks object
        
        Args:
            landmarks: BodyLandmarks object with knee, ankle, foot
            
        Returns:
            AngleResult
        """
        # Validate landmarks
        if not landmarks:
            return AngleResult(
                angle_degrees=0.0,
                raw_angle=0.0,
                is_valid=False,
                error_message="No landmarks provided"
            )
        
        # Extract points (using normalized coordinates)
        knee_pt = (landmarks.left_knee.x, landmarks.left_knee.y)
        ankle_pt = (landmarks.left_ankle.x, landmarks.left_ankle.y)
        foot_pt = (landmarks.left_foot.x, landmarks.left_foot.y)
        
        return AngleCalculator.calculate_angle(knee_pt, ankle_pt, foot_pt)
    
    @staticmethod
    def constrain_angle(angle: float, 
                       min_angle: float = 0.0,
                       max_angle: float = 180.0) -> float:
        """
        Constrain angle to valid range
        
        Args:
            angle: Input angle
            min_angle: Minimum valid angle
            max_angle: Maximum valid angle
            
        Returns:
            Constrained angle
        """
        return np.clip(angle, min_angle, max_angle)
    
    @staticmethod
    def apply_calibration_offset(angle: float, 
                                offset: float) -> float:
        """
        Apply calibration offset to angle
        
        Args:
            angle: Raw angle
            offset: Calibration offset (degrees)
            
        Returns:
            Compensated angle
        """
        return angle + offset
    
    @staticmethod
    def calculate_angle_change(angle1: float, 
                              angle2: float) -> float:
        """
        Calculate change in angle (velocity-like metric)
        
        Args:
            angle1: Previous angle
            angle2: Current angle
            
        Returns:
            Angle change
        """
        return angle2 - angle1
