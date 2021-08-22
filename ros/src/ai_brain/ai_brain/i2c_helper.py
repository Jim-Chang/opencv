import smbus

I2C_CH = 0

MICRO_CONTROLLER_ADDR = 0x50
DEFAULT_REGISTER = 0x01

CMD_DOMAIN_MOTOR = 0x01
CMD_DOMAIN_FRONT_DISTANCE = 0x02

CMD_DISTANCE_MAP = {
    'front': CMD_DOMAIN_FRONT_DISTANCE,
}

class I2CHelper:
    
    def __init__(self, channel = I2C_CH):
        self.i2c = smbus.SMBus(channel)
        
    def write(self, cmd):
        if cmd.startswith('MOTOR'):
            self._write_motor_cmd(cmd)
        
        elif cmd.startswith('U_SENSOR'):
            self._run_u_sensor_cmd(cmd)
        
        else:
            print('This cmd is not support.', cmd)

    def read(self):
        return self.i2c.read_word_data(MICRO_CONTROLLER_ADDR, DEFAULT_REGISTER)
        
    def _write_motor_cmd(self, cmd):
        '''
        str cmd: MOTOR:left:{left}:right:{right}
        i2c cmd: [CMD_DOMAIN_MOTOR, LEFT_SIGN, LEFT_SPEED, RIGHT_SIGN, RIGHT_SPEED]
        '''
        data = cmd.split(':')
        lspeed = int(data[2])
        rspeed = int(data[4])
        lsign = int(lspeed >= 0)
        rsign = int(rspeed >= 0)
        self.i2c.write_block_data(
            MICRO_CONTROLLER_ADDR,
            DEFAULT_REGISTER,
            [CMD_DOMAIN_MOTOR, lsign, abs(lspeed), rsign, abs(rspeed)]
        )

    def _run_u_sensor_cmd(self, cmd):
        """
        str cmd: U_SENSOR:front
        i2c cmd: [CMD_DOMAIN_FRONT_DISTANCE]
        """
        data = cmd.split(':')
        direction = data[1]

        i2c_cmd = CMD_DISTANCE_MAP.get(direction)
        if i2c_cmd:
            self.i2c.write_block_data(
                MICRO_CONTROLLER_ADDR,
                DEFAULT_REGISTER,
                [i2c_cmd]
            )
        else:
            print(f'{cmd} is not support')
            
