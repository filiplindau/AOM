"""
Created 2019-12-04

@author: Filip Lindau

Control of AD9910 evaluation board with RPi using SPI and some GPIO pins
"""

import spidev
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


class AD9910Control(object):
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

    def __init__(self, sys_clk=1e9, f_out=150e6):
        self.pins = {"SCLK": 40,
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
                     "P_2": 37}

        self.sys_clk = sys_clk
        self.f_out = f_out
        self.spi = None
        self.setup_gpio()
        # self.open()
        self.reset_dds()
        self.init_dds()
        self.set_frequency(f_out)

    def setup_gpio(self):
        """
        Setup gpio pins input/output

        :return:
        """
        logger.info("Setting up GPIO pins")
        gpio.setmode(gpio.BOARD)
        gpio.setup(self.pins["SCLK"], gpio.OUT)
        gpio.setup(self.pins["SDO"], gpio.OUT)
        gpio.setup(self.pins["SDI"], gpio.IN)
        gpio.setup(self.pins["IO_UPDATE"], gpio.OUT)
        gpio.setup(self.pins["IO_RESET"], gpio.OUT)
        gpio.setup(self.pins["RAM_SWP_OVR"], gpio.IN)
        gpio.setup(self.pins["EXT_PWR_DOWN"], gpio.OUT)
        gpio.setup(self.pins["MASTER_RESET"], gpio.OUT)
        gpio.setup(self.pins["PLL_LOCK"], gpio.IN)
        gpio.setup(self.pins["P_0"], gpio.OUT)
        gpio.setup(self.pins["P_1"], gpio.OUT)
        gpio.setup(self.pins["P_2"], gpio.OUT)

    def open(self):
        if self.spi is not None:
            self.close()
        logger.info("Opening SPI connection")
        self.spi = spidev.SpiDev()
        self.spi.open(0, 0)
        self.spi.max_speed_hz = 70000000

    def close(self):
        logger.info("Closing SPI connection")
        self.spi.close()
        gpio.cleanup()

    def reset_dds(self):
        logger.info("Master reset")
        self._toggle_pin("MASTER_RESET")
        time.sleep(10e-6)

    def init_dds(self):
        reg = "CFR1"
        data = [0b11100000, 0b00000000, 0b00000000, 0b00000010]
        data = [0b11100000, 0b00000000, 0b00000000, 0b00000000]
        self._write(reg, data)

        reg = "CFR2"
        data = [0b0000000, 0b01000000, 0b00000000, 0b00000000]  # SYNC_CLK enable
        self._write(reg, data)

        reg = "CFR3"
        data = [0b0000000, 0b00000000, 0b11000000, 0b00000000]  # REFCLK inpuit div bypass + REFCLK input div resetB
        self._write(reg, data)

        self._io_update()

    def set_sys_clk(self, sys_clk):
        self.sys_clk = sys_clk
        self.set_frequency(self.f_out)

    def set_frequency(self, f_out):
        logger.info("Setting output frequency {0} Hz".format(f_out))
        if f_out > self.sys_clk / 2:
            logger.warning("Frequency higher than nyquist frequency, aliasing will occur")
        ftw = np.uint32((2**32 - 1) * (f_out / self.sys_clk))
        self._set_ftw(ftw)

    def set_single_tone(self, n_profile, f_out, amplitude=1.0, phase=0.0):
        ftw = np.uint32((2 ** 32 - 1) * (f_out / self.sys_clk))

        asf = np.uint16((2**16 - 1) * amplitude)
        ph = np.uint16(phase/(2*np.pi) * (2**16 - 1))
        data = [(asf >> 8), (asf & 0xff), (ph >> 8), (ph & 0xff),
                (ftw >> 24), (ftw >> 16) & 0xff, (ftw >> 8) & 0xff, ftw & 0xff]
        reg = "PROFILE{0}".format(n_profile)
        self._write(reg, data)

    def set_phase(self, phase):
        logger.info("Setting output phase to {0} radians".format(phase))
        pow = np.uint16(phase * (2**16 - 1) / (2 * np.pi))
        self._set_pow(pow)

    def set_ram_profile(self, n_profile, start, end, step_time, mode="CONTINUOUS_RECIRCULATE"):
        """
        Setup RAM profile

        :param n_profile: Profile number to set: 0-7
        :param start: Starting RAM address 0-1023
        :param end: Ending RAM address 0-1023
        :param step_time: Time between RAM steps, seconds
        :param mode: Sweep mode: "CONTINUOUS_RECIRCULATE", "RAMP-UP", "BIDIRECTIONAL_RAMP",
        "CONTINUOUS_BIDIRECTIONAL_RAMP", or "DIRECT_SWITCH"
        :return:
        """
        if isinstance(step_time, float):
            step_time = int(step_time * self.sys_clk / 4)
        sr_high = (step_time >> 8) & 0xff
        sr_low = step_time & 0xff
        end_high = (end >> 2) & 0xff
        end_low = (end & 0b11) * 0x40
        start_high = (start >> 2) & 0xff
        start_low = (start & 0b11) * 0x40
        if mode == "CONTINUOUS_RECIRCULATE":
            mode_bits = 0b100
        elif mode == "RAMP-UP":
            mode_bits = 0b001
        elif mode == "BIDIRECTIONAL_RAMP":
            mode_bits = 0b010
        elif mode == "CONTINUOUS_BIDIRECTIONAL_RAMP":
            mode_bits = 0b001
        else:
            mode_bits = 0b000   # DIRECT_SWITCH
        data = [0, sr_high, sr_low, end_high, end_low, start_high, start_low, mode_bits]
        reg = "PROFILE{0}".format(n_profile)
        self._write(reg, data)
        self._io_update()

    def enable_ram(self, enable=True):
        logger.info("Setting RAM enable to {0}".format(enable))
        reg = "CFR1"
        data = self._read(reg, 4)
        if enable:
            # If PLL is disable, just set that bit and exit
            data[0] = data[0] | 0b10000000
        else:
            data[0] = data[0] & 0b01111111

        self._write(reg, data)

    def set_pll(self, pll_enable, pll_multiplier, vco_sel=7, ch_pump_current=7, input_div_bypass=True):
        logger.info("Set_pll: PLL enable {0}, mult {1}, vco_sel {2}, "
                    "ch_pump_current {3}, input_dev_bypass {4}".format(pll_enable, pll_multiplier, vco_sel,
                                                                       ch_pump_current, input_div_bypass))
        reg = "CFR3"
        data = self._read(reg, 4)
        if not pll_enable:
            # If PLL is disable, just set that bit and exit
            data[1] = data[1] & 0b11111110
            self._write(reg, data)
            self._io_update()
            return

        data[0] = pll_multiplier * 0b00000010
        data[1] = data[1] & 0b01111110 + 0b00000001 + input_div_bypass * 0b10000000
        data[2] = ch_pump_current * 0b00001000
        data[3] = data[3] & 0b11111100 + vco_sel
        self._write(reg, data)
        self._io_update()

    def set_aux_dac(self, aux_dac):
        """
        Set the aux_dac value that scales the output current according to:
        I_out = 86.4/R_set * (1 + aux_dac/96)

        :param aux_dac: uint8 value, default 127
        :return:
        """
        reg = "AUX_DAC_CONTROL"
        data = aux_dac
        self._write(reg, data)

    def ram_setup(self, ram_enable, ram_dest="polar"):
        # RAM setup is in the MS byte
        # We set the RAM internal profile control mode to disabled (0b0000)
        reg = "CFR1"
        data = self._read(reg, 4)
        r_data = data[0] & 0b00011111
        # RAM enable:
        if ram_enable:
            r_data = r_data | 0b10000000
        # RAM playback destination
        if ram_dest == "polar":
            r_data = r_data | 0b01100000
        elif ram_dest == "frequency":
            r_data = r_data | 0b00000000
        elif ram_dest == "phase":
            r_data = r_data | 0b00100000
        elif ram_dest == "amplitude":
            r_data = r_data | 0b01000000
        else:
            raise(ValueError("RAM destination must be polar, frequency, phase, or amplitude"))
        data[0] = r_data
        self._write(reg, data)

    def load_ram(self, data):
        reg = "RAM"
        if len(data) > 1024:
            raise(IndexError("Max ram size is 1024, got {0}".format(len(data))))
        w_data = list()
        for d in reversed(data):
            w_data.extend([(d >> 24), (d >> 16) & 0xff, (d >> 8) & 0xff, d & 0xff])
        self._write(reg, w_data)

    def _set_ftw(self, ftw):
        data = [(ftw >> 24), (ftw >> 16) & 0xff, (ftw >> 8) & 0xff, ftw & 0xff]
        reg = "FTW"
        self._write(reg, data)
        self._io_update()

    def _set_pow(self, pow):
        data = [(pow >> 8) & 0xff, pow & 0xff]
        reg = "POW"
        self._write(reg, data)
        self._io_update()

    def _write(self, register, data):
        self._write_reg(register, data)

    def _read(self, register, n_bytes):
        return self._read_reg(register, n_bytes)

    def _write_spi(self, register, data):
        """
        Write data to a register

        :param register: Name or address of register
        :param data: Data to write as a list of bytes
        :return:
        """
        logger.debug("Writing to register {0}: {1}".format(register, data))
        if isinstance(register, str):
            w_data = [self.registers[register]]
        elif isinstance(register, int):
            w_data = [register]
        else:
            raise (ValueError("Register must be string name or int address"))
        # Select write register:
        self._toggle_pin("IO_RESET")
        # self.spi.writebytes(w_data)
        # # Write data to register:
        w_data = [int(x) for x in np.uint8(data)]
        self.spi.writebytes(w_data)

        for x in data:
            w_data.append(int(np.uint8(x)))
        self.spi.writebytes(w_data)

    def _write_reg(self, register, data):
        """
        Write data to a register

        :param register: Name or address of register
        :param data: Data to write as a list of bytes
        :return:
        """
        logger.debug("Writing to register {0}: {1}".format(register, data))
        if isinstance(register, str):
            w_data = [self.registers[register]]
        elif isinstance(register, int):
            w_data = [register]
        else:
            raise (ValueError("Register must be string name or int address"))
        # Select write register:
        self._toggle_pin("IO_RESET")
        time.sleep(10e-6)
        self._write_bytes(w_data, stall_high=True)
        time.sleep(10e-6)
        # Write data to register:
        w_data = [int(np.uint8(x)) for x in data]
        self._write_bytes(w_data, stall_high=True)
        time.sleep(10e-6)
        self._toggle_pin("IO_UPDATE")

    def _read_spi(self, register, n_bytes):
        """
        Read data from register

        :param register: Name or address of register
        :param n_bytes: Number of bytes to read
        :return: List of bytes
        """
        logger.debug("Reading {1} bytes from register {0}".format(register, n_bytes))
        READ = 0x80
        if isinstance(register, str):
            w_data = [READ | self.registers[register]]
        elif isinstance(register, int):
            w_data = [READ | register]
        else:
            raise (ValueError("Register must be string name or int address"))
        # Select read register:
        logger.debug("Read instruction data: {0}".format(w_data))
        self._toggle_pin("IO_RESET")
        self.spi.writebytes(w_data)
        # Read data from register:
        data = self.spi.readbytes(n_bytes)
        return data

    def _read_reg(self, register, n_bytes):
        """
        Read data from register

        :param register: Name or address of register
        :param n_bytes: Number of bytes to read
        :return: List of bytes
        """
        logger.debug("Reading {1} bytes from register {0}".format(register, n_bytes))
        READ = 0x80
        if isinstance(register, str):
            w_data = [READ | self.registers[register]]
        elif isinstance(register, int):
            w_data = [READ | register]
        else:
            raise (ValueError("Register must be string name or int address"))
        # Select read register:
        logger.debug("Read instruction data: {0}".format(w_data))
        self._toggle_pin("IO_RESET")
        time.sleep(10e-6)
        self._write_bytes(w_data, stall_high=True)
        time.sleep(10e-6)
        # Read data from register:
        data = self._read_bytes(n_bytes)
        return data

    def _write_bytes(self, data, stall_high=False):
        logger.debug("Writing data {0}".format([hex(d) for d in data]))
        clk = self.pins["SCLK"]
        sdo = self.pins["SDO"]
        gpio.setup(sdo, gpio.OUT)
        for d in data:
            for bit in range(8):
                gpio.output(clk, 0)
                if d & 2 ** (7 - bit):
                    gpio.output(sdo, 1)
                else:
                    gpio.output(sdo, 0)
                gpio.output(clk, 1)
        if not stall_high:
            gpio.output(clk, 0)

    def _read_bytes(self, n_bytes):
        logger.debug("Reading {0} bytes".format(n_bytes))
        clk = self.pins["SCLK"]
        sdi = self.pins["SDO"]
        gpio.setup(sdi, gpio.IN)
        time.sleep(100e-6)
        data = list()
        for k in range(n_bytes):
            d = 0
            for bit in range(8):
                gpio.output(clk, 0)
                gpio.output(clk, 1)
                d += gpio.input(sdi) * 2 ** (7 - bit)
            data.append(d)
        # gpio.setup(sdi, gpio.OUT)
        return data

    def _io_update(self):
        """
        Toggle IO update pin, thereby latching register values to dds core.
        :return:
        """
        logger.info("IO Update")
        self._toggle_pin("IO_UPDATE")

    def _toggle_pin(self, pin):
        """
        Toggle pin LOW-HIGH-LOW

        :param pin: Name of pin as defined in pins dictionary
        :return:
        """
        gpio.output(self.pins[pin], 0)
        # time.sleep(1e-6)
        gpio.output(self.pins[pin], 1)
        # time.sleep(100e-6)
        gpio.output(self.pins[pin], 0)


if __name__ == "__main__":
    ac = AD9910Control(1e9, 150e6)
