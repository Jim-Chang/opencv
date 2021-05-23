from machine import PWM, Pin

_FULL_DUTY = 65534

FORWARD = 0
BACKWARD = 1

class OneWayMotor:

    def __init__(self, pin: int, freq: int = 60):
        self.pwm = PWM(Pin(pin))
        self.pwm.freq(freq)
        self.pwm.duty_u16(0)

    # speed: 0 ~ 100
    def set_speed(self, speed: int):
        self.pwm.duty_u16(int(_FULL_DUTY * (speed / 100)))


class TwoWayMotor:

    def __init__(self, pin_forward: int, pin_backward: int, freq: int = 60):
        self.pwm_f = PWM(Pin(pin_forward))
        self.pwm_b = PWM(Pin(pin_backward))
        self.pwm_f.duty_u16(0)
        self.pwm_b.duty_u16(0)

    # speed: 0 ~ 100
    def set_speed(self, speed: int, direction: int):
        if direction == FORWARD:
            self.pwm_f.duty_u16(int(_FULL_DUTY * (speed / 100)))
            self.pwm_b.duty_u16(0)

        else:
            self.pwm_f.duty_u16(0)
            self.pwm_b.duty_u16(int(_FULL_DUTY * (speed / 100)))


class TwinMotorFacade:

    def __init__(self, left_motor: TwoWayMotor, right_motor: TwoWayMotor):
        self.lmotor = left_motor
        self.rmotor = right_motor

    def set_speed(self, lspeed: int, rspeed: int):
        print('set_speed', lspeed, rspeed)
        self.lmotor.set_speed(abs(lspeed), FORWARD if lspeed > 0 else BACKWARD)
        self.rmotor.set_speed(abs(rspeed), FORWARD if rspeed > 0 else BACKWARD)
