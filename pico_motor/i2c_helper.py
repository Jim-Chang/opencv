from i2c_responser import I2CResponder

# define
CMD_DOMAIN_MOTOR = 0x01
CMD_MOTOR_L_SIGN_IDX = 1
CMD_MOTOR_L_SPEED_IDX = 2
CMD_MOTOR_R_SIGN_IDX = 3
CMD_MOTOR_R_SPEED_IDX = 4

I2C_BEGIN = 0
I2C_CHECK_DATA_COUNT = 1
I2C_GRANT_DATA = 2

class I2CSlaveHelper:

    def __init__(self, channel: int, sda_pin: int, scl_pin: int, addr: int):
        self.i2c = I2CResponder(channel, sda_pin, scl_pin, addr)
        self.motor_facade = None

    def set_motor_facade(self, motor_facade):
        self.motor_facade = motor_facade

    def check_if_need_response(self):
        # only for master check slave is exist use
        if self.i2c.read_is_pending():
            print('master need response, return 0xFF')
            self.i2c.put_read_data(0xFF)
            return True

        return False

    def check_receive(self):
        data = []
        while self.i2c.write_data_is_available():
            data += self.i2c.get_write_data(50)

        if data:
            self._handle_receive(data)
            return True

        return False

    def _handle_receive(self, data):
        print('receive data:', data)
        status = I2C_BEGIN
        data_count = 0
        cmd_temp = []
        cmds = []

        for d in data:
            if status == I2C_BEGIN:
                status = I2C_CHECK_DATA_COUNT
            
            elif status == I2C_CHECK_DATA_COUNT:
                data_count = d
                status = I2C_GRANT_DATA

            elif status == I2C_GRANT_DATA:
                if data_count > 0:
                    cmd_temp.append(d)
                    data_count -= 1

                    if data_count == 0:
                        cmds.append(cmd_temp)
                        cmd_temp = []
                        status = I2C_BEGIN                    

        print('cmds:', cmds)
        self._handle_cmds(cmds)
        
    def _handle_cmds(self, cmds):
        for cmd in cmds:
            if cmd[0] == CMD_DOMAIN_MOTOR:
                self._handle_motor_cmd(cmd)

            else:
                print('This command is not support', cmd)

    def _handle_motor_cmd(self, cmd):
        '''
        motor cmd: 
            [CMD_DOMAIN_MOTOR, LEFT_SIGN, LEFT_SPEED, RIGHT_SIGN, RIGHT_SPEED]
            [1, 0, 20, 1, 30]
        '''
        lsign = cmd[CMD_MOTOR_L_SIGN_IDX]
        rsign = cmd[CMD_MOTOR_R_SIGN_IDX]
        lspeed = cmd[CMD_MOTOR_L_SPEED_IDX]
        rspeed = cmd[CMD_MOTOR_R_SPEED_IDX]
        self.motor_facade.set_speed(
            lspeed if lsign else -lspeed,
            rspeed if rsign else -rspeed,
        )