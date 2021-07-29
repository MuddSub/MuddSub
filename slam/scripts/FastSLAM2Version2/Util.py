import numpy as np

def wrapToPi(th):
    th = np.fmod(th, 2*np.pi)
    if th >= np.pi:
        th -= 2*np.pi
    if th <= -np.pi:
        th += 2*np.pi
    return th