class PicoMotor:
    
    _MIN_SPEED = 50       # 一般最低轉速
    _MAX_SPEED = 100
    _MIN_DIFF = 10
    _MIN_DIFF_SPEED = 35  # 差速最低轉速
    
    def __init__(self, serial_port):
        self.serial_port = serial_port

    def _send(self, left, right):
        cmd = f'MOTOR:left:{left}:right:{right}'.encode()
        self.serial_port.write(cmd)
    
    def _scale_speed(self, speed, min=_MIN_SPEED, max=_MAX_SPEED):
        scale_value = int(abs(speed) / 100.0 * (max - min) + min)
        return scale_value if speed >= 0 else -scale_value

    def _get_diff_speed(self, speed, diff):
        value = abs(speed) - diff
        value = value if value >= 0 else 0

        return value if speed > 0 else -value
    
    def _is_need_change_to_rotate(self, speed, diff_speed):
        return (
            speed == diff_speed or
            ((abs(speed) - abs(diff_speed)) < self._MIN_DIFF and abs(diff_speed) == self._MIN_DIFF_SPEED)
        )
    
    def go(self, speed, diff):
        '''
        high level，會依照 speed, diff 的值來決定要 直走 還是 轉彎
        speed:
            + : forward
            - : backward
        diff:
            + : right
            - : left
        '''
        if speed == 0:
            return self.stop()
            
        if diff == 0:
            return self.go_straight(speed)
        
        return self.go_turn(speed, diff)
        
    def go_straight(self, speed):
        '''
        Always 直走
        speed:
            + : forward
            - : backward
        '''
        print(f'[PicoMotor]: go_straight, speed={speed}')
        speed = self._scale_speed(speed)
        print(f'[PicoMotor]: final => speed={speed}')
        self._send(speed, speed)
        
    def go_turn(self, speed, diff):
        '''
        轉彎專用
        speed:
            + : forward
            - : backward
        diff:
            + : right
            - : left
        '''
        print(f'[PicoMotor]: go_turn, speed={speed}, diff={diff}')
        if speed == 0:
            return
        
        speed = self._scale_speed(speed)
        diff_speed = self._scale_speed(self._get_diff_speed(speed, abs(diff)), min=self._MIN_DIFF_SPEED)
        print(f'[PicoMotor]: final => speed={speed}, diff_speed={diff_speed}')
        
        if self._is_need_change_to_rotate(speed, diff_speed):
            if (speed > 0 and diff >= 0) or (speed < 0 and diff < 0):
                print('[PicoMotor]: change to right_rotate')
                self.right_rotate()
            else:
                print('[PicoMotor]: change to left_rotate')
                self.left_rotate()
        else:    
            if diff >= 0:
                self._send(speed, diff_speed)
            else:
                self._send(diff_speed, speed)

    def turn_left(self, speed, diff):
        '''
        明確左轉使用
        speed: 
            + : forward
            - : backward
        diff:
            always +
        '''
        print(f'[PicoMotor]: turn_left, speed={speed}, diff={diff}')
        return self.go_turn(speed, -diff)

    def turn_right(self, speed, diff):
        '''
        明確右轉使用
        speed: 
            + : forward
            - : backward
        diff:
            always +
        '''
        print(f'[PicoMotor]: turn_tight, speed={speed}, diff={diff}')
        return self.go_turn(speed, diff)

    def left_rotate(self):
        '''
        原地向左旋轉
        '''
        print(f'[PicoMotor]: left_rotate')
        self._send(-self._MIN_SPEED, self._MIN_SPEED)
        
    def right_rotate(self):
        '''
        原地向右旋轉
        '''
        print(f'[PicoMotor]: right_rotate')     
        self._send(self._MIN_SPEED, -self._MIN_SPEED)
        
    def stop(self):
        self._send(0, 0)
