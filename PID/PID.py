from __future__ import annotations
from numpy import ndarray
from typing import Tuple, overload

get_sign = lambda reverse: -1.0 if reverse else 1.0
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
  
  lag_steps: int = 0
  steps: int = 0
  
  output: float = 0.0
  
  reverse: bool
  
  def __init__(self, kp: float, ki: float, kd: float, setpoint: float, update_frequency: int | float, t: float, upper_bound: float, lower_bound: float, lag_steps: int, reverse: bool = False):
    self.kp = kp
    self.ki = ki
    self.kd = kd
    self.setpoint = setpoint
    self.update_frequency = update_frequency
    self.dt = 1 / update_frequency
    self.t_last = t
    self.upper_bound = upper_bound
    self.lower_bound = lower_bound
    self.lag_steps = lag_steps
    self.reverse = reverse
  
  def step(self, t: float, state_variable: float) -> float | None:
    if t - self.t_last >= self.dt:
      self.de = ((self.setpoint - state_variable) - self.e) / (t - self.t_last)
      self.ie += (self.e + (self.setpoint - state_variable)) / 2 * (t - self.t_last) # trapezoidal integration
      self.e = (self.setpoint - state_variable)
      self.t_last = t
      self.output = self.kp * self.e + self.ki * self.ie + self.kd * self.de
      self.steps += 1
      return None
    
    if self.steps >= self.lag_steps:
      self.steps = 0
      if self.output > self.upper_bound:
        return self.upper_bound * get_sign(self.reverse)
      elif self.output < self.lower_bound:
        return self.lower_bound * get_sign(self.reverse)
      else:
        return self.output * get_sign(self.reverse)
    
    return None
  
  def set_target(self, new_target: float) -> None:
    self.setpoint = new_target