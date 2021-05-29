from machine import Pin
from define import *
from motor import TwoWayMotor, TwinMotorFacade, FORWARD
from uart_helper import UARTHelper
from i2c_helper import I2CSlaveHelper
import utime

DELAY = 0.05
led = Pin(ONBOARD_LED_PIN, Pin.OUT)

def wait_jetson_boot():
    enable = Pin(WAIT_JETSON_PIN, Pin.IN, Pin.PULL_UP)
    if enable.value() == 0:
        print('Enable wait jetson boot. Sleep 10 sec.')
        led.value(0)
        for i in range (20):
            led.toggle()
            utime.sleep(0.5)

    led.value(1)

def main():
    # init motors
    lmotor = TwoWayMotor(MOTOR_L_F_PIN, MOTOR_L_B_PIN, MOTOR_PWM_FREQ)
    rmotor = TwoWayMotor(MOTOR_R_F_PIN, MOTOR_R_B_PIN, MOTOR_PWM_FREQ)
    motor_facade = TwinMotorFacade(lmotor, rmotor)

    # init uart
    # uart = UARTHelper(UART_CHANNEL, UART_BAUDRAT, UART_TX_PIN, UART_RX_PIN)
    # uart.set_motor_facade(motor_facade)

    # init i2c
    i2c = I2CSlaveHelper(I2C_SL_CH, I2C_SL_SDA_PIN, I2C_SL_SCL_PIN, I2C_SL_ADDR)
    i2c.set_motor_facade(motor_facade)

    print('waiting for command...')
    while True:
        # uart.read_and_execute()
        r1 = i2c.check_if_need_response()
        r2 = i2c.check_receive()
        
        if r1 or r2:
            led.value(0)
            utime.sleep(0.015)

        led.value(1)
        utime.sleep(DELAY)            

if __name__ == '__main__':
    wait_jetson_boot()
    main()
