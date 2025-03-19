from __future__ import annotations
from numpy import array, pi, cos

from Differentials.Differentials import f, M, g, L
from Plotter.Plotter import plot, animate_inverted_pendulum
from Integrators.Integrators import RK4, Verlet
from PID.PID import PID
from TVC.TVC import TVC

DEGREES_TO_RADIANS = pi / 180.0
RADIANS_TO_DEGREES = 180.0 / pi

UPPER_LIMIT = 30.0
LOWER_LIMIT = -30.0

SKIPS = 4

tvc = TVC(
  max_degrees_per_sec=90.0,
  accuracy=5e-1,
  upper_bound=UPPER_LIMIT,
  lower_bound=LOWER_LIMIT,
  setpoint=0.0
)

Kp_angle, Ki_angle, Kd_angle = 0.5, 0.0, 0.5
Kp_position, Ki_position, Kd_position = 13.0, 0.0, 15.0

angular_pid = PID(
  kp=Kp_angle,
  ki=Ki_angle,
  kd=Kd_angle,
  setpoint=0.0,
  update_frequency=100,
  t=0.0,
  upper_bound=UPPER_LIMIT,
  lower_bound=LOWER_LIMIT,
  lag_steps=1,
  reverse=True
)

positional_pid = PID(
  kp=Kp_position,
  ki=Ki_position,
  kd=Kd_position,
  setpoint=0.0,
  update_frequency=100,
  t=0.0,
  upper_bound=UPPER_LIMIT,
  lower_bound=LOWER_LIMIT,
  lag_steps=1,
  reverse=False
)

t = 0.0
dt = 1e-3
tf = 10.0
ts = []
s_data = []
phi_data = []
phi_dot_data = []
s_dot_data = []

response1 = 0.0
response2 = 0.0
servo_out: float = tvc.step(response1 + response2, dt)
  
T = lambda r: 1 / 6 * M * L ** 2 * r[0, 1] ** 2 + 1 / 2 * M * r[0, 1] * r[1, 1] * L * cos(r[0, 0]) + 1 / 2 * M * r[1, 1] ** 2
U = lambda r: 1 / 2 * M * g * L * (1 - cos(r[0, 0]))
E = lambda r: T(r) + U(r)

r = array([[pi + 20 * DEGREES_TO_RADIANS, 0.0], [0.0, 0.0]], dtype=float)
responses = []

while t < tf:
  ts.append(t)
  responses.append((response1 + response2) * DEGREES_TO_RADIANS)
  phi_data.append(r[0, 0])
  phi_dot_data.append(r[0, 1])
  s_data.append(r[1, 0])
  s_dot_data.append(r[1, 1])
  
  if (n:=(angular_pid.step(t, (r[0, 0] - pi) * RADIANS_TO_DEGREES))) != None:
    response1 = n
  
  if (n:=(positional_pid.step(t, r[1, 0]))) != None:
    response2 = n
  
  if response1 + response2 > 30.0:
    response1 = 0
    response2 = 30.0
  
  servo_out = tvc.step(response1 + response2, dt)
  
  r += Verlet(f, r, t, dt, servo_out * DEGREES_TO_RADIANS)
  
  t += dt

#plot(ts, (responses, "PID"), (phi_data, "phi"), (phi_dot_data, "phi dot"), (s_data, "s"), (s_dot_data, "s dot"))
animate_inverted_pendulum(ts[::SKIPS], phi_data[::SKIPS], s_data[::SKIPS], responses[::SKIPS], L, save=False)