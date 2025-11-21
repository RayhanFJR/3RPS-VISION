#==================================================================
# FILE 1: vision/src/utils/Calibration.py
#==================================================================

import json
import os
from typing import Dict, Optional
from datetime import datetime

class CalibrationManager:
    """
    Manage calibration data and offset calculation
    """
    
    def __init__(self, config_dir: str = "./config"):
        """
        Initialize calibration manager
        
        Args:
            config_dir: Directory to store calibration files
        """
        self.config_dir = config_dir
        self.calibration_file = os.path.join(config_dir, "calibration.json")
        
        # Create config directory if not exists
        os.makedirs(config_dir, exist_ok=True)
        
        # Load existing calibration or create default
        self.calibration = self._load_calibration()
    
    def _load_calibration(self) -> Dict:
        """Load calibration from file or create default"""
        if os.path.exists(self.calibration_file):
            try:
                with open(self.calibration_file, 'r') as f:
                    return json.load(f)
            except Exception as e:
                print(f"[Calibration] Error loading file: {e}")
                return self._default_calibration()
        else:
            return self._default_calibration()
    
    def _default_calibration(self) -> Dict:
        """Create default calibration data"""
        return {
            'version': '1.0',
            'created': datetime.now().isoformat(),
            'last_updated': datetime.now().isoformat(),
            
            'angle_offset': 0.0,
            'target_angle': 90.0,
            
            'calibration_method': 'none',
            'raw_angle_at_zero': None,
            'reference_angle': None,
            
            'camera': {
                'width': 1280,
                'height': 720,
                'fps': 30,
                'camera_id': 0
            },
            
            'notes': 'Default calibration - run calibration procedure'
        }
    
    def save_calibration(self) -> bool:
        """Save calibration to file"""
        try:
            self.calibration['last_updated'] = datetime.now().isoformat()
            with open(self.calibration_file, 'w') as f:
                json.dump(self.calibration, f, indent=2)
            print(f"[Calibration] Saved to {self.calibration_file}")
            return True
        except Exception as e:
            print(f"[Calibration] Error saving: {e}")
            return False
    
    def calibrate_from_reference(self, measured_angle: float, 
                                reference_angle: float) -> float:
        """
        Calculate calibration offset from reference measurement
        
        Args:
            measured_angle: Measured foot angle
            reference_angle: Known/expected angle (e.g., 90° when standing)
            
        Returns:
            Calculated offset
        """
        offset = reference_angle - measured_angle
        
        self.calibration['angle_offset'] = float(offset)
        self.calibration['raw_angle_at_zero'] = float(measured_angle)
        self.calibration['reference_angle'] = float(reference_angle)
        self.calibration['calibration_method'] = 'reference'
        
        self.save_calibration()
        
        print(f"[Calibration] Calibrated")
        print(f"  Measured: {measured_angle:.1f}°")
        print(f"  Reference: {reference_angle:.1f}°")
        print(f"  Offset: {offset:.1f}°")
        
        return offset
    
    def calibrate_from_samples(self, samples: list,
                              reference_angle: float = 90.0) -> float:
        """
        Calculate calibration from multiple samples
        
        Args:
            samples: List of measured angles
            reference_angle: Expected angle
            
        Returns:
            Calculated offset (average)
        """
        if not samples:
            print("[Calibration] No samples provided")
            return 0.0
        
        import numpy as np
        avg_angle = np.mean(samples)
        offset = reference_angle - avg_angle
        
        self.calibration['angle_offset'] = float(offset)
        self.calibration['raw_angle_at_zero'] = float(avg_angle)
        self.calibration['reference_angle'] = float(reference_angle)
        self.calibration['calibration_method'] = 'samples'
        
        self.save_calibration()
        
        print(f"[Calibration] Calibrated from {len(samples)} samples")
        print(f"  Average measured: {avg_angle:.1f}°")
        print(f"  Reference: {reference_angle:.1f}°")
        print(f"  Offset: {offset:.1f}°")
        
        return offset
    
    def apply_calibration(self, angle: float) -> float:
        """
        Apply calibration offset to raw angle
        
        Args:
            angle: Raw measured angle
            
        Returns:
            Calibrated angle
        """
        offset = self.get_angle_offset()
        return angle + offset
    
    def get_angle_offset(self) -> float:
        """Get current angle offset"""
        return self.calibration.get('angle_offset', 0.0)
    
    def get_target_angle(self) -> float:
        """Get target angle for control"""
        return self.calibration.get('target_angle', 90.0)
    
    def set_target_angle(self, angle: float) -> None:
        """Set new target angle"""
        self.calibration['target_angle'] = float(angle)
        self.save_calibration()
    
    def get_calibration_info(self) -> Dict:
        """Get calibration information"""
        return {
            'method': self.calibration.get('calibration_method'),
            'offset': self.get_angle_offset(),
            'target': self.get_target_angle(),
            'raw_at_zero': self.calibration.get('raw_angle_at_zero'),
            'reference': self.calibration.get('reference_angle'),
            'last_updated': self.calibration.get('last_updated')
        }
    
    def reset_calibration(self) -> None:
        """Reset to default calibration"""
        self.calibration = self._default_calibration()
        self.save_calibration()
        print("[Calibration] Reset to defaults")
