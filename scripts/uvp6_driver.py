#!/usr/bin/env python
import rospy
from uvp6.msg import HWconfMsg, ACQconfMsg, LPMDataMsg, BlackDataMsg
from uvp6 import UVP6  # Make sure uvp6.py is in the same package or properly installed

class UVP6DriverNode:
    def __init__(self):
        # Initialize the node
        rospy.init_node('uvp6_driver')

        # Retrieve serial connection cfg from the parameter server or use default values
        port = rospy.get_param('~port', '/dev/ttyUSB0')
        baudrate = rospy.get_param('~baudrate', 9600)

        # Create a UVP6 object
        self.uvp6 = UVP6(port, baudrate)  # Adjust the port and baud rate as needed

        # Publishers for each message type
        self.hwconf_pub = rospy.Publisher('hwconf', HWconfMsg, queue_size=10)
        self.acqconf_pub = rospy.Publisher('acqconf', ACQconfMsg, queue_size=10)
        self.lpmdata_pub = rospy.Publisher('lpm_data', LPMDataMsg, queue_size=10)
        self.blackdata_pub = rospy.Publisher('black_data', BlackDataMsg, queue_size=10)

    def publish_data(self, msg_type, data):
        if msg_type == "HW_CONF":
            self.hwconf_pub.publish(data)
        elif msg_type == "ACQ_CONF":
            self.acqconf_pub.publish(data)
        # Add similar conditions for LPM_DATA and BLACK_DATA

    def run(self):
        # Start data acquisition - adjust the parameters as needed
        self.uvp6.start_acquisition("some_parameter_set")

        # Main loop
        while not rospy.is_shutdown():
            # Read serial data, parse it and publish
            try:
                serial_data = self.uvp6.ser.readline().decode().strip()
                if serial_data:
                    msg_type, parsed_data = self.uvp6.parse_message(serial_data)
                    self.publish_data(msg_type, parsed_data)
            except Exception as e:
                rospy.logerr("Error reading from UVP6: %s", e)
                break

        # Stop data acquisition
        self.uvp6.stop_acquisition()

        # Close serial connection
        self.uvp6.close_connection()

if __name__ == '__main__':
    try:
        node = UVP6DriverNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
