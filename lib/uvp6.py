import serial
import time

class UVP6:
    def __init__(self):
        self.ser = None

    def connect(self, port='/dev/ttyUSB0', baudrate=38400):
         """Establishes the serial connection."""
         try:
             self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
             time.sleep(2)  # Wait for the serial connection to initialize
             print("Serial connection established.")
         except serial.SerialException as e:
             print(f"Failed to establish serial connection: {e}")

    def send_command(self, command):
        """Send a command to the UVP6 instrument."""
        try:
            self.ser.write(f'{command}\r\n'.encode())
        except serial.SerialException as e:
            print(f"Serial communication error: {e}")
            self.reconnect()  # Attempt to reconnect
            try:
                self.ser.write(f'{command}\r\n'.encode())
            except Exception as e:
                print(f"Failed to send command after reconnecting: {e}")
        except Exception as e:
            print(f"Unexpected error: {e}")

    def start_acquisition(self, acq_conf, date=None, time=None): # TODO: command doesnt work, returns $starterr:51
        """Start data acquisition (Ex. Format $start:ACQ_XX,20220223,040051;)."""
        if date and time:
            command = f'$start:{str(acq_conf).zfill(2)},{date},{time};'
        else:
            command = f'$start:{str(acq_conf).zfill(2)};'
        return self.send_command(command)

    def stop_acquisition(self):
        """Stop data acquisition."""
        return self.send_command('$stop;')

    def hwconf_check(self):
        """Perform check of hardware configuration."""
        return self.send_command('$hwconfcheck;')

    def conf_check(self, acq_conf=1):
        """Perform check of aquisition configuration, specify 1-10.""" # TODO: command doesnt work, returns $starterr:51
        command = f'$confcheck:ACQ_{str(acq_conf).zfill(2)};'
        return self.send_command(command)

    def taxo_check(self, taxo_conf=1):
        """Perform check of taxonomy configuration, specify 1-10.""" # TODO: command doesnt work, returns $starterr:30
        command = f'$taxocheck:TAXO_{str(taxo_conf).zfill(2)};'
        return self.send_command(command)

    def auto_check(self):
        """Perform autocheck."""
        return self.send_command('$autocheck;')

    def instrument_check(self, acq_conf=1, taxo_conf=1):
        """Perform instrument check sequence. Verify configuration"""
        self.hwconf_check()
        time.sleep(0.5)
        self.conf_check(acq_conf)
        time.sleep(0.5)
        self.taxo_check(taxo_conf)
        time.sleep(0.5)
        self.auto_check()

    def read_response(self):
        """Read response from UVP6 instrument."""
        response = self.ser.readline().decode().strip()
        return self.message_handler(response)

    def message_handler(self, message):
        """Handle messages from UVP6"""
        response_handlers = {
            "$starterr:33;": self.handle_error,
            "$starterr:44;": self.handle_error,
            "$stopack;": self.handle_stop_ack,
            "$startack;": self.handle_start_ack,
            "$autocheckpassed;": self.handle_autocheck,
            "HW_CONF": self.parse_hwconf,
            "ACQ_CONF": self.parse_acqconf,
            "TAXO_CONF": self.parse_taxoconf,
            "LPM_DATA": self.parse_lpm_data,
            "BLACK_DATA": self.parse_black_data,
            "TAXO_DATA": self.parse_taxo_data
        }
        for key in response_handlers:
            if message.startswith(key):
                return key, response_handlers[key](message)
        return "UNKNOWN", None

    def handle_error(self, message):
        """Handle different types of errors"""
        if "starterr:33;" in message:
            print("Error 33: Instrument is busy/sleepy.")
            return self.stop_acquisition()
        elif "starterr:44;" in message:
            print("Error 44: Overexposed error.")

    def handle_stop_ack(self, message):
        print("Acquisition stopped successfully.")

    def handle_start_ack(self, message):
        print("Acquisition started successfully.")

    def handle_autocheck(self, message):
        print("Autocheck passed.")

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
            "Taxo_ids": [int(id) for id in taxoconf_data[5:]]
        }
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
                self.stop_acquisition()
                self.ser.close()
                print("Serial connection closed successfully.")
            except Exception as e:
                print(f"Error closing serial connection: {e}")
        else:
            print("Serial connection is already closed.")
