import smbus2 as smbus
import time
import numpy as np


# Oversampling options
OSR_256 = 0
OSR_512 = 1
OSR_1024 = 2
OSR_2048 = 3
OSR_4096 = 4
OSR_8192 = 5

# MS5837 hex addresses
MS5837_ADDR = 0x76
MS5837_RESET = 0x1E
MS5837_ADC_READ = 0x00
MS5837_PROM_READ = 0xA0
MS5837_CONVERT_D1 = 0x40
MS5837_CONVERT_D2 = 0x50


class PressureSensor(object):
    osr = OSR_8192
    prom = []

    def __init__(self,  device=1, address=MS5837_ADDR, osr=OSR_8192, verbose=False):
        self.verbose = verbose
        self.bus = smbus.SMBus(device)
        self.dev = device
        self.addr = address
        self.set_osr(osr)
        if not self.check_crc():
            raise Exception('CRC-4 signature mismatch')
        if self.verbose:
            print('MS5837 initialized successfully')

    def set_osr(self, osr):
        if self.osr < OSR_256 or self.osr > OSR_8192:
            raise Exception('Invalid OSR value')
        if self.verbose:
            print('Setting oversampling rate {}'.format(2 ** (8 + osr)))
        self.osr = osr
        self.read_prom()
        return self

    def read_prom(self):
        # Reset the device
        if self.verbose:
            print('Reset the device')
        self.bus.write_byte(self.addr, MS5837_RESET)
        time.sleep(2)
        # Read calibration values
        if self.verbose:
            print('Read PROM data')
        self.prom = [0 for i in range(7)]
        for i in range(7):
            val = self.bus.read_word_data(self.addr, MS5837_PROM_READ + 2 * i)
            val = ((val & 0xFF) << 8) | (val >> 8)
            self.prom[i] = val

    def read_data(self, data_addr):
        self.bus.write_byte(self.addr, data_addr + 2 * self.osr)
        time.sleep(2.5e-6 * 2 ** (8 + self.osr))
        data = self.bus.read_i2c_block_data(self.addr, MS5837_ADC_READ, 3)
        return data[0] << 16 | data[1] << 8 | data[2]

    def read(self):
        c_data = self.prom
        raw_press = self.read_data(MS5837_CONVERT_D1)
        raw_temp = self.read_data(MS5837_CONVERT_D2)

        # First order convertion
        diff = raw_temp - (c_data[5] * 256)
        temperature = (2000 + np.int32(diff) * c_data[6] / 8388608)

        offset = np.int64(c_data[2] * 131072 + (c_data[4] * np.int32(diff)) / 64)
        sensitivity = np.int64(c_data[1] * 65536 + (c_data[3] * np.int32(diff)) / 128)
        # pressure = raw_press * np.int64(sensitivity) / 2097152 - np.int64(offset) / 32768

        # Second order compensation
        temperature_i = offset_i = sensitivity_i = 0
        if (temperature / 100) < 20:
            temperature_i = (11 * np.int32(diff) * np.int32(diff)) / 34359738368
            offset_i = (31 * (temperature - 2000) * (temperature - 2000)) / 8
            sensitivity_i = (63 * (temperature - 2000) * (temperature - 2000)) / 32

        _sensitivity = sensitivity - sensitivity_i
        _offset = offset - offset_i

        result_temp = (temperature - temperature_i) / 100.0
        result_press = (((raw_press * np.int64(_sensitivity)) / 2097152 - np.int64(_offset)) / 32768) / 100.0

        return result_temp, result_press

    def check_crc(self):
        if self.verbose:
            print('Check CRC-4')
        # duplicate PROM data
        n_prom = [*self.prom]
        n_rem = 0
        # get CRC
        crc = (n_prom[0] & 0xF000) >> 12
        n_prom[0] = n_prom[0] & 0x0FFF
        n_prom.append(0)
        for i in range(16):
            if i % 2 == 1:
                n_rem ^= ((n_prom[i >> 1]) & 0x00FF)
            else:
                n_rem ^= (n_prom[i >> 1] >> 8)
            for n_bit in range(8, 0, -1):
                if n_rem & 0x8000:
                    n_rem = (n_rem << 1) ^ 0x3000
                else:
                    n_rem = (n_rem << 1)
        n_rem = ((n_rem >> 12) & 0x000F)
        return (n_rem ^ 0x00) == crc
