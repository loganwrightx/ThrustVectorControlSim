from __future__ import annotations
from numpy import array, pi, cos, mean

from Differentials.Differentials import f, M, g, L
from Plotter.Plotter import plot, animate_inverted_pendulum
from Integrators.Integrators import RK4, Verlet
from PID.PID import PID
from TVC.TVC import TVC

mse = lambda x, target: (x - target) ** 2

DEGREES_TO_RADIANS = pi / 180.0
RADIANS_TO_DEGREES = 180.0 / pi

UPPER_LIMIT_SERVO = 25.0
LOWER_LIMIT_SERVO = -25.0

UPPER_LIMIT_TARGET = 10
LOWER_LIMIT_TARGET = -10.0

SKIPS = 40

tvc = TVC(
  max_degrees_per_sec=90.0,
  accuracy=1.5,
  upper_bound=UPPER_LIMIT_SERVO,
  lower_bound=LOWER_LIMIT_SERVO,
  setpoint=0.0
)

#Kp_angle, Ki_angle, Kd_angle = 6, 0.0, 15
Kp_angle, Ki_angle, Kd_angle = 0.64211, 0.0, 0.140105
#Kp_position, Ki_position, Kd_position = 6.0, 0.0, 9.0
Kp_position, Ki_position, Kd_position = 0.0, 0.0, 0.0

angular_pid = PID(
  kp=Kp_angle,
  ki=Ki_angle,
  kd=Kd_angle,
  setpoint=0.0,
  update_frequency=100,
  t=0.0,
  upper_bound=UPPER_LIMIT_SERVO,
  lower_bound=LOWER_LIMIT_SERVO,
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
  upper_bound=UPPER_LIMIT_TARGET,
  lower_bound=LOWER_LIMIT_TARGET,
  lag_steps=1,
  reverse=False
)

t = 0.0
dt = 1e-4
tf = 5.0
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

r = array([[pi - 1 * DEGREES_TO_RADIANS, 0.0], [0.0, 0.0]], dtype=float)
responses = []

i = 0

while t < tf:
  ts.append(t)
  responses.append((response1 + response2) * DEGREES_TO_RADIANS)
  phi_data.append(r[0, 0])
  phi_dot_data.append(r[0, 1])
  s_data.append(r[1, 0])
  s_dot_data.append(r[1, 1])
  
  if (n:=(positional_pid.step(t, r[1, 0]))) != None:
    response2 = n
  
  if (n:=(angular_pid.step(t, (r[0, 0] - pi) * RADIANS_TO_DEGREES))) != None:
    response1 = n
  
  if response1 + response2 >= 30.0:
    response1 = 30.0
    response2 = 0.0
  elif response1 + response2 <= -30.0:
    response1 = -30.0
    response2 = 0.0
  
  #if i == 5000:
  #  positional_pid.set_target(1.0)
  #
  #elif i == 10000:
  #  positional_pid.set_target(-1.0)
  #
  #elif i == 15000:
  #  positional_pid.set_target(0.0)
  #
  #elif i == 20000:
  #  positional_pid.set_target(2.0)
  #
  #elif i == 25000:
  #  positional_pid.set_target(-2.0)
  
  servo_out = tvc.step(response1 + response2, dt)
  
  r += Verlet(f, r, t, dt, servo_out * DEGREES_TO_RADIANS)
  
  t += dt
  i += 1

target_s = positional_pid.setpoint
target_phi = angular_pid.setpoint

print(f"MSE of phi: {sum([mse(_phi, target_phi) for _phi in phi_data])}")
print(f"MSE of s: {sum([mse(_s, target_s) for _s in s_data])}")

#plot(ts, (responses, "PID"), (phi_data, "phi"), (phi_dot_data, "phi dot"), (s_data, "s"), (s_dot_data, "s dot"))
animate_inverted_pendulum(ts[::SKIPS], phi_data[::SKIPS], s_data[::SKIPS], responses[::SKIPS], L, save=False)