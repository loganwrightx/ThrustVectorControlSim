from __future__ import annotations
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation
from numpy import ndarray, sin, cos

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

def animate_inverted_pendulum(t: float, phi: ndarray, s: ndarray, response: float, L: float) -> None:
  width = L * 1.5
  height = L * 1.5
  fig, ax = plt.subplots()
  
  ax.set_xlim(s[0] - width, s[0] + width)
  ax.set_ylim(-height, height)
  ax.set_aspect('equal', adjustable='datalim')
  
  rocket, = ax.plot([s[0], s[0] - L * sin(phi[0])], [0, -L * cos(phi[0])])
  thrust = ax.quiver(*(s[0], 0.0, -sin(phi[0] + response[0]), -cos(phi[0] + response[0])))
  
  def update(i):
    nonlocal rocket
    nonlocal thrust
    nonlocal ax
    
    #ax.set_xlim(s[i] - width, s[i] + width)
    
    rocket.set_data([s[i], s[i] + L * sin(phi[i])], [0, -L * cos(phi[i])])
    thrust.set_offsets([s[i], 0.0])
    thrust.set_UVC(-sin(phi[i] + response[i]), cos(phi[i] + response[i]))
    
    return (rocket, thrust)
  
  ani = FuncAnimation(fig, update, frames=len(t), interval=0.5, blit=False)
  
  plt.show()