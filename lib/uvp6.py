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
        hwconf_dict = {
            "HW_CONF": hwconf_data[0],
            "Camera_ref": hwconf_data[1],
            "Acquisition_mode": int(hwconf_data[2]),
            "Default_acquisition_configuration": hwconf_data[3],
            "Delay_after_power_up_on_time_mode": int(hwconf_data[4]),
            "Light_ref": hwconf_data[5],
            "Correction_table_activation": int(hwconf_data[6]),
            "Time_between_lighting_power_up_and_trigger": int(hwconf_data[7]),
            "Time_between_lighting_trigger_and_acquisition": int(hwconf_data[8]),
            "Pressure_sensor_ref": hwconf_data[9],
            "Pressure_offset": float(hwconf_data[10]),
            "Storage_capacity": int(hwconf_data[11]),
            "Minimum_remaining_memory_for_thumbnail_saving": int(hwconf_data[12]),
            "Baud_Rate": int(hwconf_data[13]),
            "IP_address": hwconf_data[14],
            "Black_level": int(hwconf_data[15]),
            "Shutter": int(hwconf_data[16]),
            "Gain": int(hwconf_data[17]),
            "Threshold": float(hwconf_data[18]),
            "Aa": float(hwconf_data[19]),
            "Exp": float(hwconf_data[20]),
            "Pixel_Size": float(hwconf_data[21]),
            "Image_volume": float(hwconf_data[22]),
            "Calibration_date": hwconf_data[23],
            "Last_parameters_modification": hwconf_data[24],
            "Operator_email": hwconf_data[25],
            "Size_intervals": [float(size) for size in hwconf_data[26:]]
        }
        return hwconf_dict

    def parse_acqconf(self, acqconf_frame):
        acqconf_data = acqconf_frame.split(',')
        acqconf_dict = {
            "ACQ_CONF": acqconf_data[0],
            "Configuration_name": acqconf_data[1],
            "PT_mode": int(acqconf_data[2]),
            "Acquisition_frequency": float(acqconf_data[3]),
            "Frames_per_bloc": int(acqconf_data[4]),
            "Blocs_per_PT": int(acqconf_data[5]),
            "Pressure_for_auto_start": float(acqconf_data[6]),
            "Pressure_difference_for_auto_stop": float(acqconf_data[7]),
            "Result_sending": bool(int(acqconf_data[8])),
            "Save_synthetic_data_for_delayed_request": bool(int(acqconf_data[9])),
            "Limit_lpm_detection_size": int(acqconf_data[10]),
            "Save_images": bool(int(acqconf_data[11])),
            "Vignetting_lower_limit_size": int(acqconf_data[12]),
            "Appendices_ratio": float(acqconf_data[13]),
            "Interval_for_measuring_background_noise": float(acqconf_data[14]),
            "Image_nb_for_smoothing": int(acqconf_data[15]),
            "Analog_output_activation": bool(int(acqconf_data[16])),
            "Gain_for_analog_out": float(acqconf_data[17]),
            "Minimum_object_number": int(acqconf_data[18]),
            "Maximal_internal_temperature": float(acqconf_data[19]),
            "Operator_email": acqconf_data[20],
            "SD_card_remaining_memory": int(acqconf_data[21])
        }
        return acqconf_dict

    def parse_lpm_data(self, lpm_data_frame):
        lpm_data = lpm_data_frame.split(',')
        num_classes = 18  # Number of particle classes
        lpm_data_dict = {
            "LPM_DATA": lpm_data[0],
            "Depth": float(lpm_data[1]),
            "Date": lpm_data[2],
            "Time": lpm_data[3],
            "Number_of_analyzed_images": int(lpm_data[4]),
            "Internal_temperature": float(lpm_data[5]),
            "Cumulated_number_of_objects": [int(lpm_data[i]) for i in range(6, 6 + num_classes)],
            "Mean_grey_level_of_objects": [int(lpm_data[i]) for i in range(6 + num_classes, 6 + 2*num_classes)]
        }
        return lpm_data_dict

    def parse_black_data(self, black_data_frame):
        black_data = black_data_frame.split(',')
        num_classes = 18  # Number of particle classes
        black_data_dict = {
            "BLACK_DATA": black_data[0],
            "Depth": float(black_data[1]),
            "Date": black_data[2],
            "Time": black_data[3],
            "Number_of_analyzed_images": int(black_data[4]),
            "Internal_temperature": float(black_data[5]),
            "Cumulated_number_of_objects": [int(black_data[i]) for i in range(6, 6 + num_classes)]
        }
        return black_data_dict

    def parse_message(self, message):
        if message.startswith("45HW_CONF"):
            message_type = "HW_CONF"
            parsed_data = self.parse_hwconf(message)
        elif message.startswith("ACQ_CONF"):
            message_type = "ACQ_CONF"
            parsed_data = self.parse_acqconf(message)
        elif message.startswith("LPM_DATA"):
            message_type = "LPM_DATA"
            parsed_data = self.parse_lpm_data(message)
        elif message.startswith("BLACK_DATA"):
            message_type = "BLACK_DATA"
            parsed_data = self.parse_black_data(message)
        else:
            message_type = "UNKNOWN"
            parsed_data = None

        return message_type, parsed_data
