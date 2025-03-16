from __future__ import annotations
from numpy import ndarray

def RK4(f: function, r: ndarray, t: float, dt: float, response: float = None) -> ndarray:
  k1 = f(r, t, response)
  k2 = f(r + k1 * dt / 2, t + dt / 2, response)
  k3 = f(r + k2 * dt / 2, t + dt / 2, response)
  k4 = f(r + k3, t + dt, response)
  return dt * (k1 + 2 * k2 + 2 * k3 + k4) / 6

def Verlet(f: function, r: ndarray, t: float, dt: float, response: float = None) -> ndarray:
  r_half = f(r, t, response)
  r_full = f(r + dt / 2 * r_half, t + dt / 2, response)
  return dt * r_full