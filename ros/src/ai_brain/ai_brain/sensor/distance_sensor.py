class DistanceSensor:

    def __init__(self, serial_port):
        self.serial_port = serial_port

    def get_front_distance(self):
        """
        回傳單位為 mm
        """
        self.serial_port.write('U_SENSOR:front')
        return self.serial_port.read()