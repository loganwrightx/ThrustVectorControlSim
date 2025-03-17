from __future__ import annotations
from numpy import array, pi, cos

from Differentials.Differentials import f, k, M, g, L
from Plotter.Plotter import plot, animate_inverted_pendulum
from Integrators.Integrators import RK4, Verlet
from PID.PID import PID
from TVC.TVC import TVC

DEGREES_TO_RADIANS = pi / 180.0
RADIANS_TO_DEGREES = 180.0 / pi

UPPER_LIMIT = 30.0
LOWER_LIMIT = -30.0

SKIPS = 2

tvc = TVC(
  max_degrees_per_sec=90.0,
  accuracy=5e-1,
  upper_bound=UPPER_LIMIT,
  lower_bound=LOWER_LIMIT,
  setpoint=0.0
)

Kp, Ki, Kd = 0.0, 0.0, 0.0

pid = PID(
  kp=Kp,
  ki=Ki,
  kd=Kd,
  setpoint=0.0,
  update_frequency=100,
  t=0.0,
  upper_bound=UPPER_LIMIT,
  lower_bound=LOWER_LIMIT,
  lag_steps=1,
  reverse=True
)

t = 0.0
dt = 1e-3
tf = 10.0
ts = []
s_data = []
phi_data = []
phi_dot_data = []
s_dot_data = []

response = 0.0
servo_out: float = tvc.step(response, dt)
  
T = lambda r: 1 / 6 * M * L ** 2 * r[0, 1] ** 2 + 1 / 2 * M * r[0, 1] * r[1, 1] * L * cos(r[0, 0]) + 1 / 2 * M * r[1, 1] ** 2
U = lambda r: 1 / 2 * M * g * L * (1 - cos(r[0, 0]))
E = lambda r: T(r) + U(r)

r = array([[pi + 90 * DEGREES_TO_RADIANS, 0.0], [0.0, 0.0]], dtype=float)
responses = []

while t < tf:
  ts.append(t)
  responses.append(response * DEGREES_TO_RADIANS)
  phi_data.append(r[0, 0])
  phi_dot_data.append(r[0, 1])
  s_data.append(r[1, 0])
  s_dot_data.append(r[1, 1])
  
  if (n:=pid.step(t, (r[0, 0] - pi) * RADIANS_TO_DEGREES)) != None:
    response = n
  
  servo_out = tvc.step(response, dt)
  
  r += Verlet(f, r, t, dt, servo_out * DEGREES_TO_RADIANS)
  
  t += dt

#plot(ts, (responses, "PID"), (phi_data, "phi"), (phi_dot_data, "phi dot"), (s_data, "s"), (s_dot_data, "s dot"))
animate_inverted_pendulum(ts[::SKIPS], phi_data[::SKIPS], s_data[::SKIPS], responses[::SKIPS], L)