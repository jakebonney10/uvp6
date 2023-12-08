#!/usr/bin/env python3
import rospy
from uvp6_msgs.msg import HwConf, AcqConf, LpmData, BlackData
from uvp6 import UVP6

class UVP6DriverNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('uvp6_driver')

        # Retrieve settings from parameter server or use default values
        port = rospy.get_param('~port', '/dev/ttyUSB0')
        baudrate = rospy.get_param('~baudrate', 38400)
        self.acq_conf =  rospy.get_param('~acq_conf', 1)

        # Create a UVP6 object
        self.uvp6 = UVP6()
        self.uvp6.connect(port, baudrate) # initialize serial connection
        rospy.sleep(10)

        # Publishers for each message type
        self.hwconf_pub = rospy.Publisher('hw_conf', HwConf, queue_size=10)
        self.acqconf_pub = rospy.Publisher('acq_conf', AcqConf, queue_size=10)
        self.lpmdata_pub = rospy.Publisher('lpm_data', LpmData, queue_size=10)
        self.blackdata_pub = rospy.Publisher('black_data', BlackData, queue_size=10)

        # Run instrument check sequence
        #self.instrument_check()

    def instrument_check(self):
        """Perform instrument check sequence. Verify config"""
        self.uvp6.rtc_read()
        self.uvp6.read_response()
        self.uvp6.rtc_set()
        self.uvp6.read_response()
        self.uvp6.hwconf_check()
        self.uvp6.read_response()
        self.uvp6.conf_check(self.acq_conf)
        self.uvp6.read_response()
        self.uvp6.auto_check()
        self.uvp6.read_response()

    def publish_data(self, msg_type, parsed_data):
        # Publish data based on the message type
        if msg_type == "HW_CONF":
            msg = HwConf()
            self.hwconf_pub.publish(msg)
        elif msg_type == "ACQ_CONF":
            msg = AcqConf()
            self.acqconf_pub.publish(msg)
        elif msg_type == "LPM_DATA":
            msg = LpmData()
            self.lpmdata_pub.publish(msg)
        elif msg_type == "BLACK_DATA":
            msg = BlackData()
            self.blackdata_pub.publish(msg)

    def run(self):
        # Start data acquisition - choose AcqConf
        self.uvp6.start_acquisition(self.acq_conf)
        self.uvp6.start_acquisition(self.acq_conf)

        # Main loop
        while not rospy.is_shutdown():
            # Read serial data, parse it and publish
            rospy.sleep(1)
            try:
                msg = self.uvp6.read_response()
                rospy.loginfo(msg)
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
