from i2c_responser import I2CResponder
from instance import motor_facade, front_u_sensor

# define reg address
CMD_DOMAIN_MOTOR = 0x01
CMD_DOMAIN_FRONT_DISTANCE = 0x02

# define motor cmd index
CMD_MOTOR_L_SIGN_IDX = 1
CMD_MOTOR_L_SPEED_IDX = 2
CMD_MOTOR_R_SIGN_IDX = 3
CMD_MOTOR_R_SPEED_IDX = 4

I2C_BEGIN = 0
I2C_CHECK_DATA_COUNT = 1
I2C_GRANT_DATA = 2

class I2CHandlerSwitch:

    def __init__(self):
        self._handler_map = {}

    def add_handler(self, domain, receive_handler, response_handler):
        self._handler_map[domain] = {
            'receive': receive_handler,
            'response': response_handler,
        }

    def get_receive_handler(self, domain):
        h = self._handler_map.get(domain)
        if h:
            return h['receive']

    def get_response_handler(self, domain):
        h = self._handler_map.get(domain)
        if h:
            return h['response']


class I2CSlaveHelper:

    def __init__(self, channel: int, sda_pin: int, scl_pin: int, addr: int, handler_switch: I2CHandlerSwitch):
        self.i2c = I2CResponder(channel, sda_pin, scl_pin, addr)
        self._handler_switch = handler_switch
        self._response_fn = None

    def check_if_need_response(self):
        # only for master check slave is exist use
        if self.i2c.read_is_pending():
            if self._response_fn:
                result = self._response_fn()
                self._response_fn = None
            else:
                print('master need response, return 0xFF')
                result = [0xFF]
            
            print('i2c will response: ', result)
            for v in result:
                self.i2c.put_read_data(v)
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
            handler = self._handler_switch.get_receive_handler(cmd[0])
            if handler:
                handler(cmd)
                # subscribe next response handler if it has
                self._response_fn = self._handler_switch.get_response_handler(cmd[0])

            else:
                print('This command is not support', cmd)


# handlers
def handle_motor_cmd(cmd):
    '''
    motor cmd: 
        [CMD_DOMAIN_MOTOR, LEFT_SIGN, LEFT_SPEED, RIGHT_SIGN, RIGHT_SPEED]
        [1, 0, 20, 1, 30]
    '''
    lsign = cmd[CMD_MOTOR_L_SIGN_IDX]
    rsign = cmd[CMD_MOTOR_R_SIGN_IDX]
    lspeed = cmd[CMD_MOTOR_L_SPEED_IDX]
    rspeed = cmd[CMD_MOTOR_R_SPEED_IDX]
    motor_facade.set_speed(
        lspeed if lsign else -lspeed,
        rspeed if rsign else -rspeed,
    )


def handler_front_u_sensor_detect(cmd):
    print('Receive front u sensor detect')
    front_u_sensor.detect_distance_in_cm_to_buf()


def handler_front_u_sensor_get_value():
    mm = int(front_u_sensor.last_distance_in_cm * 10)
    return [
        (mm & 0xFF),
        (mm & 0xFF00) >> 8,
    ]


# handler switch facotry method
def build_i2c_handler_switch():
    sw = I2CHandlerSwitch()

    sw.add_handler(
        CMD_DOMAIN_MOTOR,
        handle_motor_cmd,
        None,
    )
    sw.add_handler(
        CMD_DOMAIN_FRONT_DISTANCE,
        handler_front_u_sensor_detect,
        handler_front_u_sensor_get_value,
    )

    return sw