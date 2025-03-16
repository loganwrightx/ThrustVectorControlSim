from __future__ import annotations
from matplotlib import pyplot as plt

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