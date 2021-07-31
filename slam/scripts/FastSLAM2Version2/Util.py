import numpy as np

def wrap_to_pi(th):
    th = np.fmod(th, 2*np.pi)
    if th >= np.pi:
        th -= 2*np.pi
    if th <= -np.pi:
        th += 2*np.pi
    return th