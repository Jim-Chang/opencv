from define import *
from instance import led_pin, enable_wait_pin
from i2c_helper import I2CSlaveHelper, build_i2c_handler_switch
import time

def wait_jetson_boot():
    if enable_wait_pin.value() == 0:
        print('Enable wait jetson boot. Sleep 10 sec.')
        led_pin.value(0)
        for i in range (20):
            led_pin.toggle()
            time.sleep(0.5)

    led_pin.value(1)

def main():
    # init i2c
    i2c = I2CSlaveHelper(
        I2C_SL_CH,
        I2C_SL_SDA_PIN,
        I2C_SL_SCL_PIN,
        I2C_SL_ADDR,
        build_i2c_handler_switch(),
    )

    print('waiting for command...')
    while True:
        r1 = i2c.check_receive()
        r2 = i2c.check_if_need_response()
        
        if r1 or r2:
            led_pin.value(0)
            time.sleep_ms(LED_FLASHING_DELAY_MS)

        led_pin.value(1)
        time.sleep_ms(I2C_LOOP_DELAY_MS)            

if __name__ == '__main__':
    wait_jetson_boot()
    main()
