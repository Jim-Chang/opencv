from define import *
from machine import Pin
from motor import TwoWayMotor, TwinMotorFacade
from hc_sr04 import HCSR04

# pins
led_pin = Pin(ONBOARD_LED_PIN, Pin.OUT)
enable_wait_pin = Pin(WAIT_JETSON_PIN, Pin.IN, Pin.PULL_UP)

# init motors
lmotor = TwoWayMotor(MOTOR_L_F_PIN, MOTOR_L_B_PIN, MOTOR_PWM_FREQ)
rmotor = TwoWayMotor(MOTOR_R_F_PIN, MOTOR_R_B_PIN, MOTOR_PWM_FREQ)
motor_facade = TwinMotorFacade(lmotor, rmotor)

# init ultrasonic sensors
front_u_sensor = HCSR04(U_SENSOR_TRIG_PIN, U_SENSOR_ECHO_PIN)