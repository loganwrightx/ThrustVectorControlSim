from __future__ import annotations
from numpy import array, pi, cos, mean

from Differentials.Differentials import f, M, g, L
from Plotter.Plotter import plot, animate_inverted_pendulum
from Integrators.Integrators import RK4, Verlet
from PID.PID import PID
from TVC.TVC import TVC

mse = lambda x, mu: (x - mu) ** 2

DEGREES_TO_RADIANS = pi / 180.0
RADIANS_TO_DEGREES = 180.0 / pi

UPPER_LIMIT_SERVO = 30.0
LOWER_LIMIT_SERVO = -30.0

UPPER_LIMIT_TARGET = 10.0
LOWER_LIMIT_TARGET = -10.0

SKIPS = 4

def control_loop(tf: float, pid_angle: tuple[float, float, float], pid_position: tuple[float, float, float]) -> tuple[float, float]:
  tvc = TVC(
    max_degrees_per_sec=90.0,
    accuracy=5e-1,
    upper_bound=UPPER_LIMIT_SERVO,
    lower_bound=LOWER_LIMIT_SERVO,
    setpoint=0.0
  )

  Kp_angle, Ki_angle, Kd_angle = pid_angle
  Kp_position, Ki_position, Kd_position = pid_position

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
  dt = 1e-3
  tf = tf
  ts = []
  s_data = []
  phi_data = []

  response1 = 0.0
  response2 = 0.0
  servo_out: float = tvc.step(response1 + response2, dt)

  r = array([[pi - 20 * DEGREES_TO_RADIANS, 0.0], [0.0, 0.0]], dtype=float)

  i = 0

  while t < tf:
    ts.append(t)
    phi_data.append(r[0, 0])
    s_data.append(r[1, 0])

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

    servo_out = tvc.step(response1 + response2, dt)

    r += Verlet(f, r, t, dt, servo_out * DEGREES_TO_RADIANS)

    t += dt
    i += 1

  target_s = positional_pid.setpoint
  target_phi = angular_pid.setpoint
  
  return sum([mse(_phi, target_phi) for _phi in phi_data]), sum([mse(_s, target_s) for _s in s_data])