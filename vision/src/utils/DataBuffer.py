#==================================================================
# FILE 2: vision/src/utils/DataBuffer.py
#==================================================================

from collections import deque
import numpy as np
from typing import List, Optional

class DataBuffer:
    """
    Circular buffer for smoothing and filtering data
    """
    
    def __init__(self, max_size: int = 5):
        """
        Initialize buffer
        
        Args:
            max_size: Maximum number of samples to keep
        """
        self.max_size = max_size
        self.buffer = deque(maxlen=max_size)
    
    def add(self, value: float) -> None:
        """
        Add value to buffer
        
        Args:
            value: Data value to add
        """
        self.buffer.append(float(value))
    
    def add_multiple(self, values: List[float]) -> None:
        """
        Add multiple values
        
        Args:
            values: List of values
        """
        for value in values:
            self.add(value)
    
    def get_average(self) -> Optional[float]:
        """
        Get average of buffered values
        
        Returns:
            Average value or None if empty
        """
        if len(self.buffer) == 0:
            return None
        
        return float(np.mean(self.buffer))
    
    def get_median(self) -> Optional[float]:
        """
        Get median of buffered values
        
        Returns:
            Median value or None if empty
        """
        if len(self.buffer) == 0:
            return None
        
        return float(np.median(self.buffer))
    
    def get_std_dev(self) -> Optional[float]:
        """
        Get standard deviation
        
        Returns:
            Std dev or None if empty
        """
        if len(self.buffer) < 2:
            return None
        
        return float(np.std(self.buffer))
    
    def get_latest(self) -> Optional[float]:
        """Get most recent value"""
        if len(self.buffer) == 0:
            return None
        
        return float(self.buffer[-1])
    
    def get_all(self) -> List[float]:
        """Get all buffered values"""
        return list(self.buffer)
    
    def is_full(self) -> bool:
        """Check if buffer is full"""
        return len(self.buffer) == self.max_size
    
    def is_empty(self) -> bool:
        """Check if buffer is empty"""
        return len(self.buffer) == 0
    
    def size(self) -> int:
        """Get current buffer size"""
        return len(self.buffer)
    
    def clear(self) -> None:
        """Clear all data"""
        self.buffer.clear()
    
    def get_range(self) -> tuple:
        """
        Get min and max of buffered values
        
        Returns:
            (min_value, max_value) or (None, None) if empty
        """
        if len(self.buffer) == 0:
            return (None, None)
        
        return (float(np.min(self.buffer)), float(np.max(self.buffer)))
    
    def exponential_smooth(self, alpha: float = 0.3) -> Optional[float]:
        """
        Apply exponential smoothing
        
        Args:
            alpha: Smoothing factor (0-1)
                  Lower = more smoothing
                  Higher = more responsive
            
        Returns:
            Smoothed value
        """
        if len(self.buffer) < 2:
            return self.get_latest()
        
        values = self.get_all()
        
        # Apply exponential smoothing
        smoothed = values[0]
        for value in values[1:]:
            smoothed = alpha * value + (1 - alpha) * smoothed
        
        return smoothed
    
    def detect_outliers(self, threshold: float = 2.0) -> List[int]:
        """
        Detect outliers using standard deviation
        
        Args:
            threshold: Number of std devs for outlier detection
            
        Returns:
            Indices of outlier values
        """
        if len(self.buffer) < 2:
            return []
        
        values = np.array(self.get_all())
        mean = np.mean(values)
        std = np.std(values)
        
        if std == 0:
            return []
        
        z_scores = np.abs((values - mean) / std)
        outlier_indices = np.where(z_scores > threshold)[0]
        
        return outlier_indices.tolist()
    
    def remove_outliers(self, threshold: float = 2.0) -> Optional[float]:
        """
        Remove outliers and return average
        
        Args:
            threshold: Outlier detection threshold
            
        Returns:
            Average without outliers
        """
        outlier_indices = set(self.detect_outliers(threshold))
        
        if len(outlier_indices) == 0:
            return self.get_average()
        
        values = [v for i, v in enumerate(self.get_all()) 
                 if i not in outlier_indices]
        
        if not values:
            return self.get_latest()
        
        return float(np.mean(values))