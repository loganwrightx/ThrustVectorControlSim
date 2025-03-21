from __future__ import annotations
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation
from numpy import ndarray, sin, cos, array, pi

plt.style.use("dark_background")

def plot(t, *args) -> None:
  names = []
  if len(args) == 0:
    return
  
  for arg in args:
    if len(arg) != 2:
      return
    
    data, name = arg
    
    if len(data) != len(t):
      return
    
    names.append(name)
    plt.plot(t, data)
  
  plt.legend(names)
  plt.show()
  
  return

def animate_inverted_pendulum(t: float, phi: ndarray, s: ndarray, response: float, L: float, save: bool = False, filename: str = "thrust_vector.mp4") -> None:
  width = L * 1.5
  height = L * 1.5
  fig, ax = plt.subplots()
  
  ax.set_xlim(s[0] - width, s[0] + width)
  ax.set_ylim(-height, height)
  ax.set_aspect('equal', adjustable='datalim')
  
  rocket, = ax.plot([s[0], s[0] - L * sin(phi[0])], [0, -L * cos(phi[0])])
  thrust = ax.quiver(s[0], 0.0, -0.5 * sin(phi[0] + response[0]), -0.5 * cos(phi[0] + response[0]), angles="xy", scale_units="xy", color="red")
  text = ax.text(s[0] - width * 0.9, height / 3, f"s = {s[0]:.3f} m\nφ = {phi[0] * 180.0 / pi - 180:.3f} deg\nθ = {response[0]:.3f} deg\nt = {t[0]:.3f} s")
  
  def update(i):
    nonlocal rocket
    nonlocal thrust
    nonlocal text
    nonlocal ax
    
    ax.set_xlim(s[i] - width, s[i] + width)
    
    rocket.set_data([s[i], s[i] + L * sin(phi[i])], [0, -L * cos(phi[i])])
    thrust.set_offsets(array([s[i], 0.0]))
    thrust.set_UVC(array([-sin(phi[i] + response[i])]), array([cos(phi[i] + response[i])]))
    
    text.set_x(s[i] - width * 0.9)
    text.set_text(f"s = {s[i]:.3f} m\nφ = {phi[i] * 180.0 / pi - 180:.3f} deg\nθ = {response[i]:.3f} deg\nt = {t[i]:.3f} s")
    
    return (rocket, thrust, text)
  
  ani = FuncAnimation(fig, update, frames=len(t), interval=1, blit=False)
  
  if save:
    print("Saving file...")
    ani.save(filename, writer="ffmpeg", fps=250)
    print("Saved successfully!")
  
  plt.show()