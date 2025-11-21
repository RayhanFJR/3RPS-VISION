#==================================================================
# FILE 4: vision/src/utils/ConfigLoader.py
#==================================================================

import json
import os
from typing import Dict, Any, Optional

class ConfigLoader:
    """
    Load configuration from files or environment variables
    """
    
    def __init__(self, config_file: str = "./config/settings.json"):
        """
        Initialize config loader
        
        Args:
            config_file: Path to config JSON file
        """
        self.config_file = config_file
        self.config = self._load_config()
    
    def _load_config(self) -> Dict[str, Any]:
        """Load configuration from file or use defaults"""
        if os.path.exists(self.config_file):
            try:
                with open(self.config_file, 'r') as f:
                    return json.load(f)
            except Exception as e:
                print(f"[Config] Error loading file: {e}")
                return self._default_config()
        else:
            print(f"[Config] File not found, using defaults")
            return self._default_config()
    
    def _default_config(self) -> Dict[str, Any]:
        """Default configuration"""
        return {
            'camera': {
                'id': 0,
                'width': 1280,
                'height': 720,
                'fps': 30
            },
            'mediapipe': {
                'model_complexity': 1,
                'min_detection_confidence': 0.5,
                'min_tracking_confidence': 0.5
            },
            'angle': {
                'offset': -35.0,
                'target': 90.0
            },
            'pid': {
                'kp': 0.5,
                'ki': 0.05,
                'kd': 0.1
            },
            'processing': {
                'angle_buffer_size': 5,
                'enable_logging': True,
                'enable_display': True
            }
        }
    
    def get(self, key: str, default: Any = None) -> Any:
        """
        Get config value with dot notation
        
        Args:
            key: Config key (e.g., "camera.fps")
            default: Default value if not found
            
        Returns:
            Config value or default
        """
        keys = key.split('.')
        value = self.config
        
        for k in keys:
            if isinstance(value, dict):
                value = value.get(k)
                if value is None:
                    return default
            else:
                return default
        
        return value
    
    def get_camera_config(self) -> Dict:
        """Get camera configuration"""
        return self.config.get('camera', {})
    
    def get_mediapipe_config(self) -> Dict:
        """Get MediaPipe configuration"""
        return self.config.get('mediapipe', {})
    
    def get_pid_gains(self) -> Dict:
        """Get PID gains"""
        return self.config.get('pid', {})
    
    def set_value(self, key: str, value: Any) -> None:
        """Set config value (dot notation)"""
        keys = key.split('.')
        config = self.config
        
        for k in keys[:-1]:
            if k not in config:
                config[k] = {}
            config = config[k]
        
        config[keys[-1]] = value
    
    def save(self) -> bool:
        """Save config to file"""
        try:
            os.makedirs(os.path.dirname(self.config_file), exist_ok=True)
            with open(self.config_file, 'w') as f:
                json.dump(self.config, f, indent=2)
            print(f"[Config] Saved to {self.config_file}")
            return True
        except Exception as e:
            print(f"[Config] Error saving: {e}")
            return False
    
    def print_config(self) -> None:
        """Print config (for debugging)"""
        print("[Config] Current configuration:")
        print(json.dumps(self.config, indent=2))