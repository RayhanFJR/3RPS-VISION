#==================================================================
# FILE 4: vision/src/control/PIDController.py
#==================================================================

from dataclasses import dataclass

@dataclass
class PIDGains:
    """PID controller gains"""
    kp: float  # Proportional
    ki: float  # Integral
    kd: float  # Derivative


class PIDController:
    """
    Simple PID controller for angle feedback
    """
    
    def __init__(self, kp: float = 0.5, 
                 ki: float = 0.05, 
                 kd: float = 0.1):
        """
        Initialize PID controller
        
        Args:
            kp: Proportional gain
            ki: Integral gain
            kd: Derivative gain
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        
        self.prev_error = 0.0
        self.integral = 0.0
        self.integral_limit = 100.0  # Prevent integral windup
    
    def compute(self, setpoint: float, 
               measurement: float, 
               dt: float = 1.0) -> float:
        """
        Compute PID output
        
        Args:
            setpoint: Desired value (target angle)
            measurement: Current value (measured angle)
            dt: Time step (default 1.0)
            
        Returns:
            PID output (control signal)
        """
        # Calculate error
        error = setpoint - measurement
        
        # Proportional term
        p_term = self.kp * error
        
        # Integral term
        self.integral += error * dt
        self.integral = max(-self.integral_limit, 
                           min(self.integral, self.integral_limit))
        i_term = self.ki * self.integral
        
        # Derivative term
        if dt > 0:
            derivative = (error - self.prev_error) / dt
        else:
            derivative = 0.0
        d_term = self.kd * derivative
        
        # Total output
        output = p_term + i_term + d_term
        
        # Store for next iteration
        self.prev_error = error
        
        return output
    
    def reset(self) -> None:
        """Reset controller state"""
        self.prev_error = 0.0
        self.integral = 0.0
    
    def set_gains(self, kp: float, ki: float, kd: float) -> None:
        """Update gains"""
        self.kp = kp
        self.ki = ki
        self.kd = kd
    
    def get_gains(self) -> PIDGains:
        """Get current gains"""
        return PIDGains(kp=self.kp, ki=self.ki, kd=self.kd)