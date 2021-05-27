from machine import Pin
from define import *
from motor import TwoWayMotor, TwinMotorFacade, FORWARD
from uart_helper import UARTHelper
import utime

DELAY = 0.05

def main():
    led = Pin(ONBOARD_LED_PIN, Pin.OUT)

    # init motors
    lmotor = TwoWayMotor(MOTOR_L_F_PIN, MOTOR_L_B_PIN, MOTOR_PWM_FREQ)
    rmotor = TwoWayMotor(MOTOR_R_F_PIN, MOTOR_R_B_PIN, MOTOR_PWM_FREQ)
    motor_facade = TwinMotorFacade(lmotor, rmotor)

    # init uart
    uart = UARTHelper(UART_CHANNEL, UART_BAUDRAT, UART_TX_PIN, UART_RX_PIN)
    uart.set_motor_facade(motor_facade)

    print('waiting for command...')
    while True:
        led.toggle()
        uart.read_and_execute()
        utime.sleep(DELAY)            

if __name__ == '__main__':
    main()
