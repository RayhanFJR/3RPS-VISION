#==================================================================
# FILE 1: vision/src/main.py - MAIN VISION PROGRAM
#==================================================================

import cv2
import sys
import time
import numpy as np
from pathlib import Path

# Add src to path
sys.path.insert(0, str(Path(__file__).parent))

from vision.PoseEstimator import PoseEstimator, BodyLandmarks
from vision.AngleCalculator import AngleCalculator
from vision.FrameProcessor import FrameProcessor
from control.PIDController import PIDController
from utils.Calibration import CalibrationManager
from utils.DataBuffer import DataBuffer
from utils.Logger import DataLogger
from utils.ConfigLoader import ConfigLoader

# ============ CONSTANTS ============
WINDOW_NAME = "Foot Angle Tracker"
CALIBRATION_SAMPLES = 30

# ============ GLOBAL STATE ============
class VisionSystem:
    """Main vision system controller"""
    
    def __init__(self, config_file: str = "./config/settings.json"):
        """Initialize vision system"""
        # Load configuration
        self.config = ConfigLoader(config_file)
        
        # Initialize components
        print("[Vision] Initializing components...")
        
        # Camera & frame processor
        camera_config = self.config.get_camera_config()
        self.frame_processor = FrameProcessor(
            camera_id=camera_config.get('id', 0),
            frame_width=camera_config.get('width', 1280),
            frame_height=camera_config.get('height', 720),
            fps=camera_config.get('fps', 30)
        )
        
        # Pose estimator
        mp_config = self.config.get_mediapipe_config()
        self.pose_estimator = PoseEstimator(
            model_complexity=mp_config.get('model_complexity', 1),
            min_detection_confidence=mp_config.get('min_detection_confidence', 0.5),
            min_tracking_confidence=mp_config.get('min_tracking_confidence', 0.5)
        )
        
        # Angle calculation
        self.angle_calculator = AngleCalculator()
        self.angle_buffer = DataBuffer(max_size=5)
        
        # Calibration
        self.calibration = CalibrationManager()
        
        # PID controller
        pid_gains = self.config.get_pid_gains()
        self.pid_controller = PIDController(
            kp=pid_gains.get('kp', 0.5),
            ki=pid_gains.get('ki', 0.05),
            kd=pid_gains.get('kd', 0.1)
        )
        
        # Logger
        self.logger = DataLogger(
            enabled=self.config.get('processing.enable_logging', True)
        )
        
        # State
        self.active_side = "LEFT"
        self.frame_count = 0
        self.last_print_time = time.time()
        self.running = True
        self.calibration_mode = False
        self.calibration_samples_left = CALIBRATION_SAMPLES
        
        print("[Vision] Initialization complete!")
        print(f"[Vision] Camera: {self.frame_processor.actual_width}x{self.frame_processor.actual_height} @ {self.frame_processor.actual_fps} FPS")
    
    def run(self):
        """Main loop"""
        print(f"\n[Vision] Starting main loop...")
        print("Controls:")
        print("  'l' = LEFT foot")
        print("  'r' = RIGHT foot")
        print("  'c' = Calibrate")
        print("  'q' = Quit")
        print()
        
        while self.running:
            ret, frame = self.frame_processor.read_frame()
            if not ret:
                print("[Vision] Failed to read frame")
                break
            
            self.frame_count += 1
            
            # Process frame
            landmarks = self.pose_estimator.process_frame(frame)
            
            # Calculate angle
            if landmarks and landmarks.is_valid():
                # Get raw angle
                angle_result = self.angle_calculator.calculate_angle_from_landmarks(landmarks)
                
                if angle_result.is_valid:
                    raw_angle = angle_result.angle_degrees
                    
                    # Smooth angle
                    self.angle_buffer.add(raw_angle)
                    smoothed_angle = self.angle_buffer.get_average()
                    
                    # Apply calibration
                    compensated_angle = self.calibration.apply_calibration(smoothed_angle)
                    
                    # Handle calibration mode
                    if self.calibration_mode:
                        self.calibration_samples_left -= 1
                        
                        if self.calibration_samples_left <= 0:
                            self.finish_calibration()
                        else:
                            self._display_calibration_status(frame)
                    else:
                        # PID control
                        target_angle = self.calibration.get_target_angle()
                        pid_output = self.pid_controller.compute(
                            target_angle, compensated_angle
                        )
                        
                        # Log data
                        self.logger.log_data(
                            self.frame_count,
                            raw_angle,
                            smoothed_angle,
                            compensated_angle,
                            pid_output,
                            side=self.active_side
                        )
                        
                        # Display
                        frame = self._add_display_info(
                            frame, raw_angle, smoothed_angle,
                            compensated_angle, pid_output, landmarks
                        )
            else:
                # No detection
                self._add_no_detection_message(frame)
            
            # Draw pose
            frame = self.pose_estimator.draw_landmarks(frame, landmarks)
            
            # Display frame
            key = self.frame_processor.display_frame(frame, WINDOW_NAME, wait_ms=1)
            
            # Handle keyboard input
            if key != -1:
                self._handle_keyboard(key)
            
            # Print statistics occasionally
            if time.time() - self.last_print_time >= 2.0:
                self._print_statistics()
                self.last_print_time = time.time()
        
        self.shutdown()
    
    def start_calibration(self):
        """Start calibration procedure"""
        self.calibration_mode = True
        self.calibration_samples_left = CALIBRATION_SAMPLES
        self.angle_buffer.clear()
        print(f"\n[Calibration] Starting... Collect {CALIBRATION_SAMPLES} samples")
    
    def finish_calibration(self):
        """Finish calibration"""
        samples = self.angle_buffer.get_all()
        
        if samples:
            self.calibration.calibrate_from_samples(
                samples,
                reference_angle=90.0  # Assume foot at 90° when standing
            )
        
        self.calibration_mode = False
        self.angle_buffer.clear()
        print("[Calibration] Complete!")
    
    def _handle_keyboard(self, key: int):
        """Handle keyboard input"""
        if key == ord('q'):
            print("\n[Vision] Quitting...")
            self.running = False
        
        elif key == ord('l'):
            self.active_side = "LEFT"
            self.angle_buffer.clear()
            print("[Vision] Switched to LEFT foot")
        
        elif key == ord('r'):
            self.active_side = "RIGHT"
            self.angle_buffer.clear()
            print("[Vision] Switched to RIGHT foot")
        
        elif key == ord('c'):
            self.start_calibration()
        
        elif key == ord('p'):
            self._print_config()
    
    def _add_display_info(self, frame, raw_angle, smoothed_angle,
                         compensated_angle, pid_output, landmarks):
        """Add display information to frame"""
        h, w = frame.shape[:2]
        
        # Main angle display
        text = f"{self.active_side} Foot: {compensated_angle:.1f}°"
        frame = self.frame_processor.add_text(
            frame, text, (10, 50),
            font_scale=1.2, thickness=2,
            color=(0, 255, 0)
        )
        
        # Smoothed angle
        text = f"Smoothed: {smoothed_angle:.1f}°"
        frame = self.frame_processor.add_text(
            frame, text, (10, 90),
            font_scale=0.8, color=(100, 100, 255)
        )
        
        # PID output
        text = f"PID: {pid_output:.2f}"
        frame = self.frame_processor.add_text(
            frame, text, (10, 130),
            font_scale=0.8, color=(255, 100, 100)
        )
        
        # Instructions
        text = "l=LEFT, r=RIGHT, c=CALIBRATE, q=QUIT"
        frame = self.frame_processor.add_text(
            frame, text, (10, h-20),
            font_scale=0.7, color=(255, 255, 0)
        )
        
        # FPS
        text = f"Frame: {self.frame_count}"
        frame = self.frame_processor.add_text(
            frame, text, (w-200, 30),
            font_scale=0.7, color=(200, 200, 200)
        )
        
        return frame
    
    def _display_calibration_status(self, frame):
        """Display calibration progress"""
        h, w = frame.shape[:2]
        
        text = f"CALIBRATION: {CALIBRATION_SAMPLES - self.calibration_samples_left}/{CALIBRATION_SAMPLES}"
        frame = self.frame_processor.add_text(
            frame, text, (w//2 - 150, h//2),
            font_scale=1.5, thickness=3,
            color=(0, 255, 255)
        )
    
    def _add_no_detection_message(self, frame):
        """Add no detection message"""
        h, w = frame.shape[:2]
        text = "No detection"
        frame = self.frame_processor.add_text(
            frame, text, (w//2 - 100, h//2),
            font_scale=1.0, color=(0, 0, 255)
        )
    
    def _print_statistics(self):
        """Print system statistics"""
        pose_stats = self.pose_estimator.get_statistics()
        frame_info = self.frame_processor.get_frame_info()
        cal_info = self.calibration.get_calibration_info()
        
        print(f"\n[Stats] Frame {self.frame_count}")
        print(f"  Detection: {pose_stats.get('detection_rate', 0):.1f}%")
        print(f"  Angle offset: {cal_info['offset']:.1f}°")
        print(f"  Target: {cal_info['target']:.1f}°")
    
    def _print_config(self):
        """Print current configuration"""
        print("\n")
        self.config.print_config()
    
    def shutdown(self):
        """Shutdown system"""
        print("\n[Vision] Shutting down...")
        self.frame_processor.close()
        self.pose_estimator.close()
        
        if self.logger.get_log_file():
            print(f"[Vision] Log saved: {self.logger.get_log_file()}")
        
        print("[Vision] Goodbye!")


# ============ MAIN ============

def main():
    """Main entry point"""
    print("=" * 50)
    print("  FOOT ANGLE VISION SYSTEM")
    print("  MediaPipe + OpenCV")
    print("=" * 50)
    
    try:
        # Initialize and run
        system = VisionSystem()
        system.run()
    
    except KeyboardInterrupt:
        print("\n[Vision] Interrupted by user")
    
    except Exception as e:
        print(f"\n[Vision] Error: {e}")
        import traceback
        traceback.print_exc()
        return 1
    
    return 0


if __name__ == "__main__":
    sys.exit(main())