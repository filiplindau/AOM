"""
Test of software controlled SPI on RPi using GPIO

Created 2019-12-10

@author: Filip Lindau
"""

import RPi.GPIO as gpio
import time
import logging
import numpy as np

logger = logging.getLogger(__name__)
while len(logger.handlers):
    logger.removeHandler(logger.handlers[0])

f = logging.Formatter("%(asctime)s - %(module)s.   %(funcName)s - %(levelname)s - %(message)s")
fh = logging.StreamHandler()
fh.setFormatter(f)
logger.addHandler(fh)
logger.setLevel(logging.DEBUG)


pins = {"SCLK": 40,
        "SDO": 38,
        "SDI": 36,
        "IO_UPDATE": 7,
        "IO_RESET": 11,
        "RAM_SWP_OVR": 13,
        "EXT_PWR_DOWN": 15,
        "MASTER_RESET": 29,
        "PLL_LOCK": 31,
        "P_0": 33,
        "P_1": 35,
        "P_2": 37
        }

registers = {"CFR1": 0x00,
             "CFR2": 0x01,
             "CFR3": 0x02,
             "AUX_DAC_CONTROL": 0x03,
             "IO_UPDATE_RATE": 0x04,
             "FTW": 0x07,
             "POW": 0x08,
             "ASF": 0x09,
             "PROFILE0": 0x0e,
             "PROFILE1": 0x0f,
             "PROFILE2": 0x10,
             "PROFILE3": 0x11,
             "PROFILE4": 0x12,
             "PROFILE5": 0x13,
             "PROFILE6": 0x14,
             "PROFILE7": 0x15,
             "RAM": 0x16,
             }


def setup_gpio():
    logger.info("Setting up GPIO pins")
    gpio.setmode(gpio.BOARD)
    gpio.setup(pins["SCLK"], gpio.OUT)
    gpio.setup(pins["SDO"], gpio.OUT)
    gpio.setup(pins["SDI"], gpio.IN)
    gpio.setup(pins["IO_UPDATE"], gpio.OUT)
    gpio.setup(pins["IO_RESET"], gpio.OUT)
    gpio.setup(pins["RAM_SWP_OVR"], gpio.IN)
    gpio.setup(pins["EXT_PWR_DOWN"], gpio.OUT)
    gpio.setup(pins["MASTER_RESET"], gpio.OUT)
    gpio.setup(pins["PLL_LOCK"], gpio.IN)
    gpio.setup(pins["P_0"], gpio.OUT)
    gpio.setup(pins["P_1"], gpio.OUT)
    gpio.setup(pins["P_2"], gpio.OUT)



def test_speed(n=1000):
    logger.info("Testing pin toggle speed for {0} cycles".format(n))
    clk = pins["SCLK"]
    for k in range(n):
        gpio.output(clk, 1)
        gpio.output(clk, 0)


def write_bytes(data, stall_high=False):
    logger.debug("Writing data {0}".format([hex(d) for d in data]))
    clk = pins["SCLK"]
    sdo = pins["SDO"]
    gpio.setup(sdo, gpio.OUT)
    for d in data:
        for bit in range(8):
            gpio.output(clk, 0)
            if d & 2**(7-bit):
                gpio.output(sdo, 1)
            else:
                gpio.output(sdo, 0)
            gpio.output(clk, 1)
    if not stall_high:
        gpio.output(clk, 0)


def read_bytes(n_bytes):
    logger.debug("Reading {0} bytes".format(n_bytes))
    clk = pins["SCLK"]
    sdi = pins["SDI"]
    data = list()
    for k in range(n_bytes):
        d = 0
        for bit in range(8):
            gpio.output(clk, 0)
            d += gpio.input(sdi) * 2**(7-bit)
            gpio.output(clk, 1)
        data.append(d)
    gpio.output(clk, 0)
    return data


def read_bytes_2w(n_bytes):
    logger.debug("Reading {0} bytes".format(n_bytes))
    clk = pins["SCLK"]
    sdi = pins["SDO"]
    gpio.setup(sdi, gpio.IN)
    time.sleep(100e-6)
    data = list()
    for k in range(n_bytes):
        d = 0
        for bit in range(8):
            gpio.output(clk, 0)
            gpio.output(clk, 1)
            d += gpio.input(sdi) * 2**(7-bit)
        data.append(d)
    # gpio.setup(sdi, gpio.OUT)
    return data


def _toggle_pin(pin):
    """
    Toggle pin LOW-HIGH-LOW

    :param pin: Name of pin as defined in pins dictionary
    :return:
    """
    gpio.output(pins[pin], 0)
    # time.sleep(1e-6)
    gpio.output(pins[pin], 1)
    # time.sleep(100e-6)
    gpio.output(pins[pin], 0)


def write_reg(register, data):
    """
    Write data to a register

    :param register: Name or address of register
    :param data: Data to write as a list of bytes
    :return:
    """
    logger.debug("Writing to register {0}: {1}".format(register, data))
    if isinstance(register, str):
        w_data = [registers[register]]
    elif isinstance(register, int):
        w_data = [register]
    else:
        raise (ValueError("Register must be string name or int address"))
    # Select write register:
    _toggle_pin("IO_RESET")
    time.sleep(10e-6)
    write_bytes(w_data, stall_high=True)
    # Write data to register:
    w_data = [int(np.uint8(x)) for x in data]
    write_bytes(w_data, stall_high=True)
    time.sleep(10e-6)
    _toggle_pin("IO_UPDATE")


def read_reg(register, n_bytes):
    """
    Read data from register

    :param register: Name or address of register
    :param n_bytes: Number of bytes to read
    :return: List of bytes
    """
    logger.debug("Reading {1} bytes from register {0}".format(register, n_bytes))
    READ = 0x80
    if isinstance(register, str):
        w_data = [READ | registers[register]]
    elif isinstance(register, int):
        w_data = [READ | register]
    else:
        raise (ValueError("Register must be string name or int address"))
    # Select read register:
    logger.debug("Read instruction data: {0}".format(w_data))
    _toggle_pin("IO_RESET")
    write_bytes(w_data, stall_high=True)
    # Read data from register:
    data = read_bytes_2w(n_bytes)
    return data
