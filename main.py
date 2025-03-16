from __future__ import annotations
from numpy import array, pi, cos

from Differentials.Differentials import f, k, M, g, L
from Plotter.Plotter import plot
from Integrators.Integrators import RK4, Verlet
from PID.PID import PID

DEGREES_TO_RADIANS = pi / 180.0
RADIANS_TO_DEGREES = 180.0 / pi

UPPER_LIMIT = 30.0
LOWER_LIMIT = -30.0

Kp, Ki, Kd = 2.5, 0.0, 0.5

pid = PID(
  kp=Kp,
  ki=Ki,
  kd=Kd,
  setpoint=180.0,
  update_frequency=10,
  t=0.0,
  upper_bound=UPPER_LIMIT,
  lower_bound=LOWER_LIMIT,
  lag_steps=1
)

response = 0.0

t = 0.0
dt = 1e-3
tf = 20.0
ts = []
s_data = []
phi_data = []
phi_dot_data = []
s_dot_data = []
  
T = lambda r: 1 / 6 * M * L ** 2 * r[0, 1] ** 2 + 1 / 2 * M * r[0, 1] * r[1, 1] * L * cos(r[0, 0]) + 1 / 2 * M * r[1, 1] ** 2
U = lambda r: 1 / 2 * M * g * L * (1 - cos(r[0, 0]))
E = lambda r: T(r) + U(r)

r = array([[pi + 1e-2, 0.0], [0.0, 0.0]], dtype=float)
responses = []

while t < tf:
  ts.append(t)
  responses.append(response * DEGREES_TO_RADIANS)
  phi_data.append(r[0, 0] - pi)
  phi_dot_data.append(r[0, 1])
  s_data.append(r[1, 0])
  s_dot_data.append(r[1, 1])
  
  if (n:=pid.step(t, r[0, 0] * RADIANS_TO_DEGREES)) != None:
    response = n
  
  r += Verlet(f, r, t, dt, response * DEGREES_TO_RADIANS)
  
  t += dt

plot(ts, (phi_data, "phi"), (responses, "PID"))