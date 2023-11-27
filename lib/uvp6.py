import serial
import time

class UVP6:
    def __init__(self, port='COM1', baudrate=9600):
        self.ser = serial.Serial(port, baudrate, timeout=1)
        time.sleep(2)  # wait for the serial connection to initialize

    def send_command(self, command):
        """Send a command to the UVP6 instrument."""
        self.ser.write(f'{command}\r\n'.encode())
        return self.read_response()

    def read_response(self):
        """Read response from UVP6 instrument."""
        response = self.ser.readline().decode().strip()
        # Additional logic to handle different types of responses
        return response

    def start_acquisition(self, parameter_set, date=None, time=None):
        """Start data acquisition."""
        if date and time:
            command = f'$start:{parameter_set},{date},{time};'
        else:
            command = f'$start:{parameter_set};'
        return self.send_command(command)

    def stop_acquisition(self):
        """Stop data acquisition."""
        return self.send_command('$stop;')

    def autocheck(self):
        """Perform autocheck of the instrument."""
        return self.send_command('$autocheck;')

    def parse_hwconf(self, hwconf_frame):
        """Parse the hardware configuration serial message from UVP6."""
        hwconf_data = hwconf_frame.split(',')
        hwconf_dict = {
            "HW_CONF": hwconf_data[0],
            "Camera_ref": hwconf_data[1],
            "Acquisition_mode": int(hwconf_data[2]),
            "Default_acquisition_configuration": hwconf_data[3],
            "Delay_after_power_up_on_time_mode": int(hwconf_data[4]),
            "Light_ref": hwconf_data[5],
            "Correction_table_activation": int(hwconf_data[6]),
            "Time_between_lighting_trigger_and_acquisition": int(hwconf_data[7]),
            "Pressure_sensor_ref": hwconf_data[8],
            "Pressure_offset": float(hwconf_data[9]),
            "Storage_capacity": int(hwconf_data[10]),
            "Minimum_remaining_memory_for_thumbnail_saving": int(hwconf_data[11]),
            "Baud_Rate": int(hwconf_data[12]),
            "Black_level": int(hwconf_data[13]),
            "Shutter": int(hwconf_data[14]),
            "Gain": int(hwconf_data[15]),
            "Threshold": float(hwconf_data[16]),
            "Aa": float(hwconf_data[17]),
            "Exp": float(hwconf_data[18]),
            "Pixel_Size": float(hwconf_data[19]),
            "Image_volume": float(hwconf_data[20]),
            "Calibration_date": hwconf_data[21],
            "Last_parameters_modification": hwconf_data[22],
            "Operator_email": hwconf_data[23],
            "Size_intervals": [float(size) for size in hwconf_data[24:]]
        }
        return hwconf_dict

    def parse_acqconf(self, acqconf_frame):
        """Parse the acquisition configuration serial message from UVP6."""
        acqconf_data = acqconf_frame.split(',')
        acqconf_dict = {
            "ACQ_CONF": acqconf_data[0],
            "Configuration_name": acqconf_data[1],
            "PT_mode": int(acqconf_data[2]),
            "Acquisition_frequency": float(acqconf_data[3]),
            "Frames_per_bloc": int(acqconf_data[4]),
            "Pressure_for_auto_start": float(acqconf_data[5]),
            "Pressure_difference_for_auto_stop": float(acqconf_data[6]),
            "Result_sending": bool(int(acqconf_data[7])),
            "Save_synthetic_data_for_delayed_request": bool(int(acqconf_data[8])),
            "Save_images": bool(int(acqconf_data[9])),
            "Vignetting_lower_limit_size": int(acqconf_data[10]),
            "Appendices_ratio": float(acqconf_data[11]),
            "Interval_for_measuring_background_noise": float(acqconf_data[12]),
            "Image_nb_for_smoothing": int(acqconf_data[13]),
            "Analog_output_activation": bool(int(acqconf_data[14])),
            "Gain_for_analog_out": float(acqconf_data[15]),
            "Maximal_internal_temperature": float(acqconf_data[16]),
            "Operator_email": acqconf_data[17],
            "Taxo_conf": acqconf_data[18],
            "Aux_mode": acqconf_data[19],
            "Aux_param_1": acqconf_data[20],
            "Aux_param_2": acqconf_data[21],
            "Remaining_memory": int(acqconf_data[22])
        }
        return acqconf_dict

    def parse_taxoconf(self, taxoconf_frame):
        """Parse the taxonomic configuration serial message from UVP6."""
        taxoconf_data = taxoconf_frame.split(',')
        taxoconf_dict = {
            "TAXO_CONF": taxoconf_data[0],
            "Configuration_name": taxoconf_data[1],
            "Model_reference": taxoconf_data[2],
            "Max_size_for_classification": int(taxoconf_data[3]),
            "Model_nb_classes": int(taxoconf_data[4]),
        }
        # Parsing Taxo IDs for each class
        num_classes = 40  # Number of taxonomic classes (adjust as needed)
        for i in range(num_classes):
            class_key = f"Taxo_ID_for_class_{str(i).zfill(2)}"
            taxoconf_dict[class_key] = int(taxoconf_data[5 + i])

        return taxoconf_dict

    def parse_lpm_data(self, lpm_data_frame):
        """Parse the LPM data from UVP6 serial message."""
        lpm_data = lpm_data_frame.split(',')
        num_classes = 18  # Number of particle classes
        lpm_data_dict = {
            "LPM_DATA": lpm_data[0],
            "Pressure": float(lpm_data[1]),
            "Date": lpm_data[2],
            "Time": lpm_data[3],
            "Number_of_analyzed_images": int(lpm_data[4]),
            "Internal_temperature": float(lpm_data[5]),
            "Nbi": [int(lpm_data[i]) for i in range(6, 6 + num_classes)],
            "GGi": [int(lpm_data[i]) for i in range(6 + num_classes, 6 + 2*num_classes)]
        }
        return lpm_data_dict

    def parse_black_data(self, black_data_frame):
        """Parse the black data from UVP6 serial message."""
        black_data = black_data_frame.split(',')
        num_classes = 18  # Number of particle classes
        black_data_dict = {
            "BLACK_DATA": black_data[0],
            "Pressure": float(black_data[1]),
            "Date": black_data[2],
            "Time": black_data[3],
            "Number_of_analyzed_images": int(black_data[4]),
            "Internal_temperature": float(black_data[5]),
            "Nbi": [int(black_data[i]) for i in range(6, 6 + num_classes)]
        }
        return black_data_dict

    def parse_taxo_data(self, taxo_data_frame):
        """Parse the TAXO_DATA message from UVP6."""
        taxo_data = taxo_data_frame.split(',')
        taxo_data_dict = {
            "IMG": int(taxo_data[1]),
            "Objects": []
        }

        # Parsing objects data
        num_fields_per_object = 3  # NNi, VVi, and GGi for each object
        num_objects = (len(taxo_data) - 2) // num_fields_per_object

        for i in range(num_objects):
            start_index = 2 + i * num_fields_per_object
            object_data = {
                "Category": int(taxo_data[start_index]),
                "Volume": int(taxo_data[start_index + 1]),
                "Grey_Level": int(taxo_data[start_index + 2])
            }
            taxo_data_dict["Objects"].append(object_data)

        return taxo_data_dict

    def parse_message(self, message):
        """Handle different serial responses from UVP6."""
        if message.startswith("45HW_CONF"):
            message_type = "HW_CONF"
            parsed_data = self.parse_hwconf(message)
        elif message.startswith("ACQ_CONF"):
            message_type = "ACQ_CONF"
            parsed_data = self.parse_acqconf(message)
        elif message.startswith("TAXO_CONF"):
            message_type = "TAXO_CONF"
            parsed_data = self.parse_taxoconf(message)
        elif message.startswith("LPM_DATA"):
            message_type = "LPM_DATA"
            parsed_data = self.parse_lpm_data(message)
        elif message.startswith("BLACK_DATA"):
            message_type = "BLACK_DATA"
            parsed_data = self.parse_black_data(message)
        elif message.startswith("TAXO_DATA"):
            message_type = "TAXO_DATA"
            parsed_data = self.parse_taxo_data(message)
        else:
            message_type = "UNKNOWN"
            parsed_data = None

        return message_type, parsed_data

    def reconnect(self):
        """Check connection and reconnect to UVP6 if needed."""
        if not self.ser.is_open:
            try:
                self.ser.open()
                print("Reconnected to the serial port.")
            except Exception as e:
                print(f"Failed to reconnect: {e}")
        else:
            print("Already connected to the serial port.")

    def close_connection(self):
        """Close connection to UVP6 and stop data aquisition for soft shutdown."""
        if self.ser.is_open:
            try:
                # add pre-closure steps here
                self.stop_acquisition()
                self.ser.close()
                print("Serial connection closed successfully.")
            except Exception as e:
                print(f"Error closing serial connection: {e}")
        else:
            print("Serial connection is already closed.")
