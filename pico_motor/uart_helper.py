from machine import UART, Pin


'''
command define:
<domain>:<command>:<data>:<command>:<data>...
'''

# domain define
DOMAIN_MOTOR = 'MOTOR'

class UARTHelper:

    def __init__(self, channel, baudrate, pin_tx, pin_rx):
        self.uart = UART(channel, baudrate=baudrate, tx=Pin(pin_tx), rx=Pin(pin_rx))
        self.motor_facade = None

    def set_motor_facade(self, motor_facade):
        self.motor_facade = motor_facade

    def read_and_execute(self):
        while self.uart.any():
            data = self.uart.readline()
            try:
                command = data.decode().replace('\n', '')
                print('UART receive command: '+ command)

                self._handle(command)

            except Exception as e:
                print('UART command error', e)

    def _handle(self, command):
        # MOTOR:left:100:right:50
        if command.startswith(DOMAIN_MOTOR):
            self._handle_motor_command(command.replace(DOMAIN_MOTOR + ':', ''))

        else:
            print('This command is not support')

    def _handle_motor_command(self, command):
        # left:100:right:50
        cmds = command.split(':')
        lspeed = int(cmds[1])
        rspeed = int(cmds[3])
        self.motor_facade.set_speed(lspeed, rspeed)
