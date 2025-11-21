#==================================================================
# FILE 3: vision/src/utils/Logger.py
#==================================================================

import csv
import os
from datetime import datetime
from typing import Optional, List

class DataLogger:
    """
    Log angle and control data to CSV file
    """
    
    def __init__(self, log_dir: str = "./logs", 
                enabled: bool = True):
        """
        Initialize logger
        
        Args:
            log_dir: Directory to store log files
            enabled: Whether logging is enabled
        """
        self.log_dir = log_dir
        self.enabled = enabled
        
        if enabled:
            os.makedirs(log_dir, exist_ok=True)
            
            # Create log filename with timestamp
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.log_file = os.path.join(
                log_dir, 
                f"foot_angle_{timestamp}.csv"
            )
            
            # Write header
            self._write_header()
        else:
            self.log_file = None
    
    def _write_header(self) -> None:
        """Write CSV header"""
        if not self.enabled or not self.log_file:
            return
        
        headers = [
            'timestamp',
            'frame_num',
            'raw_angle',
            'smoothed_angle',
            'compensated_angle',
            'pid_output',
            'side',
            'detection_confidence'
        ]
        
        try:
            with open(self.log_file, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(headers)
        except Exception as e:
            print(f"[Logger] Error writing header: {e}")
    
    def log_data(self, frame_num: int,
                raw_angle: float,
                smoothed_angle: float,
                compensated_angle: float,
                pid_output: float,
                side: str = "LEFT",
                confidence: float = 0.0) -> None:
        """
        Log data entry
        
        Args:
            frame_num: Frame number
            raw_angle: Raw calculated angle
            smoothed_angle: Smoothed angle
            compensated_angle: Angle after calibration
            pid_output: PID controller output
            side: Body side (LEFT/RIGHT)
            confidence: Detection confidence
        """
        if not self.enabled or not self.log_file:
            return
        
        timestamp = datetime.now().isoformat()
        
        row = [
            timestamp,
            frame_num,
            f"{raw_angle:.2f}",
            f"{smoothed_angle:.2f}",
            f"{compensated_angle:.2f}",
            f"{pid_output:.2f}",
            side,
            f"{confidence:.3f}"
        ]
        
        try:
            with open(self.log_file, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(row)
        except Exception as e:
            print(f"[Logger] Error logging data: {e}")
    
    def get_log_file(self) -> Optional[str]:
        """Get log file path"""
        return self.log_file if self.enabled else None
    
    def disable(self) -> None:
        """Disable logging"""
        self.enabled = False
    
    def enable(self) -> None:
        """Enable logging"""
        self.enabled = True