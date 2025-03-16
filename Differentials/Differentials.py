from __future__ import annotations
from numpy import ndarray, sin, cos, array
from numpy.random import normal as uniform_random

δs = 2.5e-2

g = 9.8
L = 1.0
M = 1.0
k = 1.0
T = 10.0

PHI, S = 0, 1

def f(r: ndarray, t: float, response: float | None = None) -> ndarray:
  """
  phi, phi_dot = r[0, 0:1]
  s, s_dot = r[1, 0:1]
  
  returns dr
  """
  phi = r[PHI, 0]
  phi_dot = r[PHI, 1]
  s = r[S, 0]
  s_dot = r[S, 1]
  
  if response == None:
    response = 0.0
  
  phi_double_dot = -3 * (g * sin(phi) + L / 2 * phi_dot ** 2 * sin(phi) * cos(phi) + \
    T * sin(phi + response) / M + δs * uniform_random()) / (2 * L * (1 - 3 / 4 * cos(phi) ** 2))
  s_double_dot = L / 2 * (phi_dot ** 2 * sin(phi) - phi_double_dot * cos(phi)) - k / M * s + T * sin(phi + response) / M + δs * uniform_random()
  
  return array([[phi_dot, phi_double_dot], [s_dot, s_double_dot]], dtype=float)