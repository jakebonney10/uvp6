import serial
import time

class UVP6:
    def __init__(self, port, baudrate=9600):
        self.ser = serial.Serial(port, baudrate, timeout=1)
        time.sleep(2)  # wait for the serial connection to initialize

    def send_command(self, command):
        self.ser.write(f'{command}\r\n'.encode())
        response = self.ser.readline().decode().strip()
        if response == "$starterr:33;":
            self.ser.write(f'{command}\r\n'.encode())  # repeat command
            response = self.ser.readline().decode().strip()
        return response

    def start_acquisition(self, parameter_set, date=None, time=None):
        if date and time:
            command = f'$start:{parameter_set},{date},{time};'
        else:
            command = f'$start:{parameter_set};'
        return self.send_command(command)

    def stop_acquisition(self):
        return self.send_command('$stop;')

    def autocheck(self):
        return self.send_command('$autocheck;')

    # Additional functions to parse HWconf, ACQconf, LPM_DATA, BLACK_DATA etc.
