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

    def parse_hwconf(self, hwconf_frame):
        hwconf_data = hwconf_frame.split(',')
        # Assuming hwconf_data follows the known format, map it to a dictionary
        hwconf_dict = {
            "HW_CONF": hwconf_data[0],
            "Camera_ref": hwconf_data[1],
            # Continue mapping all the known parameters
        }
        return hwconf_dict

    def parse_acqconf(self, acqconf_frame):
        acqconf_data = acqconf_frame.split(',')
        acqconf_dict = {
            "ACQ_CONF": acqconf_data[0],
            "Configuration_name": acqconf_data[1],
            # Continue mapping all the known parameters
        }
        return acqconf_dict

    def parse_lpm_data(self, lpm_data_frame):
        lpm_data = lpm_data_frame.split(',')
        lpm_data_dict = {
            "LPM_DATA": lpm_data[0],
            "Depth": float(lpm_data[1]),
            # Continue mapping all the known parameters
        }
        return lpm_data_dict

    def parse_black_data(self, black_data_frame):
        black_data = black_data_frame.split(',')
        black_data_dict = {
            "BLACK_DATA": black_data[0],
            "Depth": float(black_data[1]),
            # Continue mapping all the known parameters
        }
        return black_data_dict

    # Add any additional methods needed for handling responses

