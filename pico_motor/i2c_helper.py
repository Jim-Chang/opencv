from machine import Pin, I2C

class I2CSlaveHelper:

    def __init__(self, channel: int, sda_pin: int, scl_pin: int, freq: int):
        self.i2c = I2C(channel, Pin(sda_pin), Pin(scl_pin), freq)