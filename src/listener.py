#!/usr/bin/env python
"""
ROS-Arduino Communication Node

This script implements a ROS node that facilitates communication between ROS and an Arduino
board via serial connection. It subscribes to rover control commands and forwards them to 
the Arduino through the serial port.

The script can automatically detect the Arduino port or accept manual port input.

Dependencies:
    - rospy
    - std_msgs
    - pyserial

Author: Mollah Md Saif
"""

import rospy
from std_msgs.msg import String
import serial
from typing import Optional

class ArduinoCommNode:
    """
    A class to handle ROS-Arduino communication.
    
    This class manages the serial connection to an Arduino and handles ROS message
    forwarding to the device.
    
    Attributes:
        BAUDRATE (int): Serial communication baud rate
        serial_port (serial.Serial): Serial connection object
    """
    
    BAUDRATE = 9600

    def __init__(self):
        """Initialize the Arduino communication node."""
        self.serial_port: Optional[serial.Serial] = None

    def initialize_node(self) -> None:
        """
        Initialize the ROS node and set up the subscriber.
        
        This method sets up the ROS node named 'rover' and subscribes to
        the 'rover_control' topic.
        """
        rospy.init_node('rover', anonymous=True)
        rospy.Subscriber("rover_control", String, self.move_callback)
        rospy.loginfo("Rover node initialized and subscribed to rover_control")

    def move_callback(self, message: String) -> None:
        """
        Callback function for handling incoming ROS messages.
        
        Args:
            message (String): The ROS message containing control commands
        """
        try:
            self.serial_port.write(message.data.encode('utf-8'))
            rospy.loginfo(f"Sent command: {message.data}")
        except (serial.SerialException, AttributeError) as e:
            rospy.logerr(f"Failed to send command: {str(e)}")

    def serial_connect(self, auto_select: bool = False) -> None:
        """
        Establish serial connection with the Arduino.
        
        Args:
            auto_select (bool): If True, automatically tries to detect the Arduino port.
                              If False, prompts for manual port input.
        
        Raises:
            serial.SerialException: If connection to the specified port fails
        """
        if auto_select:
            if self._try_auto_connect():
                return

        self._manual_connect()

    def _try_auto_connect(self) -> bool:
        """
        Attempt to automatically detect and connect to the Arduino.
        
        Returns:
            bool: True if connection successful, False otherwise
        """
        for usb_type in ['ttyUSB', 'ttyACM']:
            for i in range(4):
                port = f'/dev/{usb_type}{i}'
                try:
                    self.serial_port = serial.Serial(port, self.BAUDRATE, timeout=0.01)
                    rospy.loginfo(f"Connected to Arduino on port {port}")
                    return True
                except serial.SerialException:
                    rospy.logwarn(f"Failed to connect to Arduino on port {port}")
        return False

    def _manual_connect(self) -> None:
        """
        Establish serial connection using manually specified port.
        
        Raises:
            serial.SerialException: If connection to the specified port fails
        """
        while not rospy.is_shutdown():
            try:
                input_port = input("Enter the port of the Arduino: ")
                self.serial_port = serial.Serial(input_port, self.BAUDRATE)
                rospy.loginfo(f"Connected to Arduino on port {input_port}")
                break
            except serial.SerialException as e:
                rospy.logerr(f"Failed to connect: {str(e)}")
                if not rospy.is_shutdown():
                    rospy.loginfo("Please try again")

    def run(self) -> None:
        """
        Main execution method.
        
        This method initializes the serial connection and starts the ROS node.
        It blocks until the node is shutdown.
        """
        try:
            self.serial_connect(auto_select=True)
            self.initialize_node()
            rospy.spin()
        except rospy.ROSInterruptException as e:
            rospy.logerr(f"Node interrupted: {str(e)}")
        finally:
            if self.serial_port and self.serial_port.is_open:
                self.serial_port.close()
                rospy.loginfo("Serial port closed")

def main():
    """Main entry point for the script."""
    node = ArduinoCommNode()
    node.run()

if __name__ == '__main__':
    main()