#==================================================================
# FILE 1: vision/src/vision/PoseEstimator.py
#==================================================================

import cv2
import mediapipe as mp
import numpy as np
from dataclasses import dataclass
from typing import Optional, Tuple

# ============ DATA STRUCTURES ============

@dataclass
class Point3D:
    """3D point from MediaPipe"""
    x: float
    y: float
    z: float
    visibility: float
    
    def to_2d(self, frame_width: int, frame_height: int) -> Tuple[float, float]:
        """Convert normalized coordinates to pixel coordinates"""
        return (self.x * frame_width, self.y * frame_height)


@dataclass
class BodyLandmarks:
    """Complete body landmarks"""
    left_knee: Point3D
    left_ankle: Point3D
    left_foot: Point3D
    
    right_knee: Point3D
    right_ankle: Point3D
    right_foot: Point3D
    
    # Additional useful landmarks
    left_hip: Optional[Point3D] = None
    right_hip: Optional[Point3D] = None
    
    def is_valid(self, min_visibility: float = 0.5) -> bool:
        """Check if all required landmarks are visible"""
        required = [
            self.left_knee, self.left_ankle, self.left_foot,
            self.right_knee, self.right_ankle, self.right_foot
        ]
        return all(lm.visibility >= min_visibility for lm in required)


# ============ CLASS DEFINITION ============

class PoseEstimator:
    """
    MediaPipe-based pose estimation for foot angle calculation
    """
    
    # MediaPipe landmark indices
    LEFT_KNEE = 25
    LEFT_ANKLE = 27
    LEFT_FOOT_INDEX = 31
    RIGHT_KNEE = 26
    RIGHT_ANKLE = 28
    RIGHT_FOOT_INDEX = 32
    LEFT_HIP = 23
    RIGHT_HIP = 24
    
    def __init__(self, 
                 model_complexity: int = 1,
                 min_detection_confidence: float = 0.5,
                 min_tracking_confidence: float = 0.5):
        """
        Initialize MediaPipe Pose estimator
        
        Args:
            model_complexity: 0 (lite) or 1 (full - more accurate)
            min_detection_confidence: Confidence threshold for detection
            min_tracking_confidence: Confidence threshold for tracking
        """
        self.model_complexity = model_complexity
        self.min_detection_confidence = min_detection_confidence
        self.min_tracking_confidence = min_tracking_confidence
        
        # Initialize MediaPipe
        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose(
            static_image_mode=False,
            model_complexity=model_complexity,
            smooth_landmarks=True,
            min_detection_confidence=min_detection_confidence,
            min_tracking_confidence=min_tracking_confidence
        )
        
        self.mp_drawing = mp.solutions.drawing_utils
        self.drawing_spec = self.mp_drawing.DrawingSpec(
            thickness=2, circle_radius=2, color=(0, 255, 0)
        )
        
        # Statistics
        self.frame_count = 0
        self.detection_count = 0
    
    def process_frame(self, frame: np.ndarray) -> Optional[BodyLandmarks]:
        """
        Process frame and extract body landmarks
        
        Args:
            frame: Input frame (BGR format from OpenCV)
            
        Returns:
            BodyLandmarks object or None if detection failed
        """
        self.frame_count += 1
        
        # Convert BGR to RGB
        image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        image.flags.writeable = False
        
        # Process with MediaPipe
        results = self.pose.process(image)
        
        image.flags.writeable = True
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        
        if not results.pose_landmarks:
            return None
        
        self.detection_count += 1
        
        # Extract landmarks
        landmarks = results.pose_landmarks.landmark
        
        try:
            body_landmarks = BodyLandmarks(
                left_knee=self._extract_landmark(landmarks[self.LEFT_KNEE]),
                left_ankle=self._extract_landmark(landmarks[self.LEFT_ANKLE]),
                left_foot=self._extract_landmark(landmarks[self.LEFT_FOOT_INDEX]),
                
                right_knee=self._extract_landmark(landmarks[self.RIGHT_KNEE]),
                right_ankle=self._extract_landmark(landmarks[self.RIGHT_ANKLE]),
                right_foot=self._extract_landmark(landmarks[self.RIGHT_FOOT_INDEX]),
                
                left_hip=self._extract_landmark(landmarks[self.LEFT_HIP]),
                right_hip=self._extract_landmark(landmarks[self.RIGHT_HIP])
            )
            
            return body_landmarks
        
        except Exception as e:
            print(f"[PoseEstimator] Error extracting landmarks: {e}")
            return None
    
    def draw_landmarks(self, frame: np.ndarray, 
                      landmarks: Optional[BodyLandmarks],
                      thickness: int = 2) -> np.ndarray:
        """
        Draw landmarks on frame
        
        Args:
            frame: Input frame
            landmarks: Body landmarks to draw
            thickness: Line thickness
            
        Returns:
            Frame with drawn landmarks
        """
        if landmarks is None:
            return frame
        
        h, w = frame.shape[:2]
        
        # Draw left leg
        self._draw_line_between_points(
            frame, landmarks.left_knee, landmarks.left_ankle, w, h, thickness
        )
        self._draw_line_between_points(
            frame, landmarks.left_ankle, landmarks.left_foot, w, h, thickness
        )
        
        # Draw right leg
        self._draw_line_between_points(
            frame, landmarks.right_knee, landmarks.right_ankle, w, h, thickness
        )
        self._draw_line_between_points(
            frame, landmarks.right_ankle, landmarks.right_foot, w, h, thickness
        )
        
        # Draw points
        color_joint = (0, 255, 0)      # Green for joints
        color_foot = (0, 0, 255)       # Red for foot
        radius = 5
        
        for pt in [landmarks.left_knee, landmarks.right_knee]:
            x, y = pt.to_2d(w, h)
            cv2.circle(frame, (int(x), int(y)), radius, color_joint, -1)
        
        for pt in [landmarks.left_ankle, landmarks.right_ankle]:
            x, y = pt.to_2d(w, h)
            cv2.circle(frame, (int(x), int(y)), radius, color_joint, -1)
        
        for pt in [landmarks.left_foot, landmarks.right_foot]:
            x, y = pt.to_2d(w, h)
            cv2.circle(frame, (int(x), int(y)), radius, color_foot, -1)
        
        return frame
    
    def _extract_landmark(self, landmark) -> Point3D:
        """Extract landmark and convert to Point3D"""
        return Point3D(
            x=landmark.x,
            y=landmark.y,
            z=landmark.z,
            visibility=landmark.visibility
        )
    
    def _draw_line_between_points(self, frame: np.ndarray,
                                  pt1: Point3D, pt2: Point3D,
                                  width: int, height: int,
                                  thickness: int) -> None:
        """Draw line between two points"""
        x1, y1 = pt1.to_2d(width, height)
        x2, y2 = pt2.to_2d(width, height)
        
        # Only draw if both points visible
        if pt1.visibility >= 0.5 and pt2.visibility >= 0.5:
            cv2.line(frame, (int(x1), int(y1)), (int(x2), int(y2)), 
                    (0, 255, 0), thickness)
    
    def get_statistics(self) -> dict:
        """Get detection statistics"""
        if self.frame_count == 0:
            return {}
        
        detection_rate = (self.detection_count / self.frame_count) * 100
        
        return {
            'total_frames': self.frame_count,
            'detected_frames': self.detection_count,
            'detection_rate': detection_rate
        }
    
    def reset_statistics(self) -> None:
        """Reset statistics counters"""
        self.frame_count = 0
        self.detection_count = 0
    
    def close(self) -> None:
        """Close pose estimator"""
        if self.pose:
            self.pose.close()
