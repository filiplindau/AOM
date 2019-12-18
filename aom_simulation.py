"""
Simulate pulse shape from input pulse and acoustic waveform

Created 2019-12-05

@author: Filip Lindau
"""

import time
import logging
import numpy as np
from scipy.fftpack import fft
import scipy.fftpack as fp


class AOMSimulation(object):
    def __init__(self):
        pass


if __name__ == "__main__":
    aomsim = AOMSimulation()
    c = 299792458.0
    t = np.linspace(-1e-12, 1e-12, 100000)
    dt = t[1] - t[0]
    w = fp.fftfreq(t.shape[0], dt)
    l = 2*np.pi*c / w
    l0 = 263e-9
    delta_t = 100e-15
    t_sig = np.sqrt(2 / np.log(2)) * delta_t
    E = np.sin(2*np.pi*c/l0 * t) * np.exp(-t**2 / t_sig**2)

    Ew = fft(E)
