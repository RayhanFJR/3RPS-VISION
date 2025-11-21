#==================================================================
# FILE 3: vision/src/vision/FrameProcessor.py
#==================================================================

import cv2
import numpy as np
from typing import Tuple, Optional

class FrameProcessor:
    """
    Handle frame processing: capture, resize, display, etc.
    """
    
    def __init__(self, camera_id: int = 0,
                 frame_width: int = 1280,
                 frame_height: int = 720,
                 fps: int = 30):
        """
        Initialize frame processor
        
        Args:
            camera_id: Camera device ID (0 for default)
            frame_width: Target frame width
            frame_height: Target frame height
            fps: Target FPS
        """
        self.camera_id = camera_id
        self.frame_width = frame_width
        self.frame_height = frame_height
        self.fps = fps
        
        # Open camera
        self.cap = cv2.VideoCapture(camera_id)
        
        if not self.cap.isOpened():
            raise RuntimeError(f"Cannot open camera {camera_id}")
        
        # Set camera properties
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, frame_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        
        # Get actual properties
        self.actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
        
        # Frame statistics
        self.frame_count = 0
    
    def read_frame(self) -> Tuple[bool, Optional[np.ndarray]]:
        """
        Read single frame from camera
        
        Returns:
            (success, frame) - frame is None if failed
        """
        ret, frame = self.cap.read()
        if ret:
            self.frame_count += 1
        return ret, frame
    
    def display_frame(self, frame: np.ndarray, 
                     window_name: str = "Frame",
                     wait_ms: int = 1) -> int:
        """
        Display frame in window
        
        Args:
            frame: Frame to display
            window_name: Window title
            wait_ms: Wait time in milliseconds
            
        Returns:
            Key pressed (or -1 if no key)
        """
        cv2.imshow(window_name, frame)
        return cv2.waitKey(wait_ms)
    
    def add_text(self, frame: np.ndarray,
                text: str,
                position: Tuple[int, int] = (10, 30),
                font_scale: float = 1.0,
                thickness: int = 2,
                color: Tuple[int, int, int] = (0, 255, 0)) -> np.ndarray:
        """
        Add text overlay to frame
        
        Args:
            frame: Input frame
            text: Text to display
            position: (x, y) position
            font_scale: Font size
            thickness: Text thickness
            color: (B, G, R) color
            
        Returns:
            Frame with text
        """
        cv2.putText(frame, text, position,
                   cv2.FONT_HERSHEY_SIMPLEX,
                   font_scale, color, thickness)
        return frame
    
    def add_angle_display(self, frame: np.ndarray,
                         angle: float,
                         title: str = "Angle") -> np.ndarray:
        """
        Add angle information overlay
        
        Args:
            frame: Input frame
            angle: Angle value
            title: Display title
            
        Returns:
            Frame with angle display
        """
        h, w = frame.shape[:2]
        
        # Main angle text (large, top-left)
        text = f"{title}: {angle:.1f}°"
        self.add_text(frame, text, (10, 50), font_scale=1.5, 
                     thickness=2, color=(0, 255, 0))
        
        # Draw angle arc/indicator (bottom-right)
        # Map angle to position
        center_x = w - 100
        center_y = h - 100
        radius = 50
        
        # Draw angle indicator circle
        cv2.circle(frame, (center_x, center_y), radius, (100, 100, 100), 2)
        
        # Draw angle indicator line
        angle_rad = np.radians(angle)
        end_x = int(center_x + radius * np.cos(angle_rad - np.pi/2))
        end_y = int(center_y + radius * np.sin(angle_rad - np.pi/2))
        cv2.line(frame, (center_x, center_y), (end_x, end_y), (0, 255, 0), 2)
        
        return frame
    
    def resize_frame(self, frame: np.ndarray,
                    width: Optional[int] = None,
                    height: Optional[int] = None) -> np.ndarray:
        """
        Resize frame maintaining aspect ratio
        
        Args:
            frame: Input frame
            width: Target width (None = keep aspect ratio)
            height: Target height (None = keep aspect ratio)
            
        Returns:
            Resized frame
        """
        if width is None and height is None:
            return frame
        
        h, w = frame.shape[:2]
        
        if width is None:
            width = int(w * height / h)
        elif height is None:
            height = int(h * width / w)
        
        return cv2.resize(frame, (width, height))
    
    def get_frame_info(self) -> dict:
        """Get frame and camera information"""
        return {
            'camera_id': self.camera_id,
            'frame_width': self.actual_width,
            'frame_height': self.actual_height,
            'fps': self.actual_fps,
            'total_frames': self.frame_count
        }
    
    def close(self) -> None:
        """Close camera and windows"""
        self.cap.release()
        cv2.destroyAllWindows()