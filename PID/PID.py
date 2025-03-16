from __future__ import annotations
from numpy import ndarray
from typing import Tuple, overload

class PID:
  kp: float
  ki: float
  kd: float
  setpoint: float
  update_frequency: int | float
  t_last: float
  dt: float
  
  e: float = 0.0
  de: float = 0.0
  ie: float = 0.0
  
  upper_bound: float
  lower_bound: float
  
  def __init__(self, kp: float, ki: float, kd: float, setpoint: float, update_frequency: int | float, t: float, upper_bound: float, lower_bound: float):
    self.kp = kp
    self.ki = ki
    self.kd = kd
    self.setpoint = setpoint
    self.update_frequency = update_frequency
    self.dt = 1 / update_frequency
    self.t_last = t
    self.upper_bound = upper_bound
    self.lower_bound = lower_bound
  
  def step(self, t: float, state_variable: float) -> float | None:
    if t - self.t_last >= self.dt:
      self.de = ((self.setpoint - state_variable) - self.e) / (t - self.t_last)
      self.ie += (self.e + (self.setpoint - state_variable)) / 2 * (t - self.t_last) # trapezoidal integration
      self.e = (self.setpoint - state_variable)
      self.t_last = t
      output = self.kp * self.e + self.ki * self.ie + self.kd * self.de
      
      if output > self.upper_bound:
        return self.upper_bound
      elif output < self.lower_bound:
        return self.lower_bound
      else:
        return output
    
    return None