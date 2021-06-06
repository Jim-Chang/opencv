import smbus

I2C_CH = 0

MOTOR_ADDR = 0x50
CMD_DOMAIN_MOTOR = 0x01

class I2CHelper:
    
    def __init__(self, channel = I2C_CH):
        self.i2c = smbus.SMBus(channel)
        
    def write(self, cmd):
        cmd = cmd.decode()
        if cmd.startswith('MOTOR'):
            self._write_motor_cmd(cmd)
        
        else:
            print('This cmd is not support.', cmd)
        
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
            MOTOR_ADDR,
            0x01,
            [CMD_DOMAIN_MOTOR, lsign, abs(lspeed), rsign, abs(rspeed)]
        )