from __future__ import annotations
from numpy.random import normal as random_normal

sign = lambda n: 1.0 if n >= 0.0 else -1.0

_z_score = 0.31864 # 25% probability of not having inaccuracies

class TVC:
  value: float
  setpoint: float
  accuracy: float
  max_rate: float
  upper_bound: float
  lower_bound: float
  
  def __init__(self, max_degrees_per_sec: float, accuracy: float, upper_bound: float, lower_bound: float, setpoint: float = 0.0):
    self.setpoint = setpoint
    self.max_rate = abs(max_degrees_per_sec)
    self.value = setpoint
    self.upper_bound = upper_bound
    self.lower_bound = lower_bound
    self.accuracy = accuracy
  
  def step(self, response: float, dt: float) -> float:
    dt = abs(dt)
    self.setpoint = response
    if abs(d:=(self.setpoint - self.value)) <= self.max_rate * dt:
      self.value = self.setpoint
    else:
      self.value += sign(d) * self.max_rate * dt
    
    if (rv:=random_normal()) > _z_score:
      return self.value + self.accuracy # introduce randomness
    elif rv < _z_score:
      return self.value - self.accuracy
    else:
      return self.value