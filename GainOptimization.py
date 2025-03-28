from __future__ import annotations
from numpy import linspace, meshgrid, zeros, unravel_index
import matplotlib.pyplot as plt
from ControlLoop import control_loop

tf = 10.0 # seconds

pid_angle = (0.61, 0.0, 0.121)
pid_position = (20.6315316, 0.0, 6.721)

N = 20

def optimize_phi_pid(pid: tuple[float, float, float], N: int = N, test_range = 1e-1, plot: bool = False) -> tuple[float, float, float]:
  P, I, D = 0, 1, 2
  
  p_vals = linspace(start=pid[P] - test_range, stop=pid[P] + test_range, num=N)
  d_vals = linspace(start=pid[D] - test_range, stop=pid_angle[D] + test_range, num=N)

  ps, ds = meshgrid(p_vals, d_vals, indexing="ij")

  phi_mses = zeros(shape=(N, N))

  h = 1

  for i, _p in enumerate(p_vals):
    for j, _d in enumerate(d_vals):
      print(f"{h}/{N * N}")
      h += 1
      phi_mses[i, j] = control_loop(tf, (_p, 0.0, _d), (0.0, 0.0, 0.0))[0]

  (i, j) = unravel_index(phi_mses.argmin(), phi_mses.shape)
  
  p_opt = ps[i, j]
  d_opt = ds[i, j]
  
  print(f"Minimized MSE of phi: {phi_mses.min()}")
  print(f"p = {p_opt}")
  print(f"d = {d_opt}")
  print(i, j)

  if plot:
    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection="3d")
    ax.plot_surface(ps, ds, phi_mses, cmap="viridis")
    ax.set_xlabel("p")
    ax.set_ylabel("d")
    plt.show()
  
  return (p_opt, 0.0, d_opt)

def optimize_s_pid(pid: tuple[float, float, float], N: int = N, test_range = 3, plot: bool = False) -> tuple[float, float, float]:
  P, I, D = 0, 1, 2
  
  p_vals = linspace(start=pid[P] - test_range, stop=pid[P] + test_range, num=N)
  d_vals = linspace(start=pid[D] - test_range, stop=pid[D] + test_range, num=N)

  ps, ds = meshgrid(p_vals, d_vals, indexing="ij")

  s_mses = zeros(shape=(N, N))

  h = 1

  for i, _p in enumerate(p_vals):
    for j, _d in enumerate(d_vals):
      print(f"{h}/{N * N}")
      h += 1
      s_mses[i, j] = control_loop(tf, (_p, 0.0, _d), (0.0, 0.0, 0.0))[1]

  (i, j) = unravel_index(s_mses.argmin(), s_mses.shape)
  
  p_opt = ps[i, j]
  d_opt = ds[i, j]
  
  print(f"Minimized MSE of phi: {s_mses.min()}")
  print(f"p = {p_opt}")
  print(f"d = {d_opt}")
  print(i, j)

  if plot:
    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection="3d")
    ax.plot_surface(ps, ds, s_mses, cmap="viridis")
    ax.set_xlabel("p")
    ax.set_ylabel("d")
    plt.show()
  
  return (p_opt, 0.0, d_opt)

if __name__ == "__main__":
  optimize_s_pid(pid_position, test_range=4, plot=True)