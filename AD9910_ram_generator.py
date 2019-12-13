"""
Generate RAM file to use with AD9910EVAL board software

Created 2019-12-03

@author: Filip Lindau
"""
import numpy as np


def generate_polar_ram(r, th):
    """
    Create a RAM vector that can be written to the AD9910 with polar modulation.
    16 bit phase, 14 bit amplitude

    :param r: Array with normalized amplitude values, max length 1024
    :param th: Array with phase values (radians), max length 1024 (same as r)
    :return:
    """
    if max(r) > 1 or min(r) < 0:
        raise ValueError("Amplitudes must be in range 0 - 1")
    ram_list = list()
    for k in range(r.shape[0]):
        # a = np.uint32(np.uint16(r[k] * 2**14) * 2**16)
        # ph = np.uint16(th[k]/(2*np.pi)*2**16)
        a = np.uint16(r[k] * 2**14) * 4
        ph = np.uint32(np.uint16(th[k]/(2*np.pi)*2**16) * 2**16)
        ram_list.append(a + ph)
    return np.array(ram_list)


if __name__ == "__main__":
    # Test with a parabolic amplitude curve
    x = np.linspace(0, 1, 200)
    r = (2*(x - 0.5))**2
    th = np.zeros_like(r)
    a = generate_polar_ram(r, th)
    np.savetxt("parabol_polar3.txt", a, "%d")
