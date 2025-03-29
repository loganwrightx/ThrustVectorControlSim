from __future__ import annotations
from numpy import ndarray, sin, cos, array, tanh
from numpy.random import normal as uniform_random

δs = 2.5e-1

g = 9.8
L = 0.85
M = 0.636
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
  
  response = 0.3 * tanh(response * 3.5) # models the nonlinear actuation which actually occurs in the mechanism
  
  a_s = (T * sin(phi + response) + 5 * uniform_random()) / M # additional driving forces in s direction divided by mass
  phi_double_dot = -3 * (g * sin(phi) + (L / 2 * phi_dot ** 2 * sin(phi) + a_s) * cos(phi)) / (2 * L * (1 - 3 / 4 * cos(phi) ** 2))
  s_double_dot = L / 2 * (phi_dot ** 2 * sin(phi) - phi_double_dot * cos(phi)) + a_s
  
  return array([[phi_dot, phi_double_dot], [s_dot, s_double_dot]], dtype=float)