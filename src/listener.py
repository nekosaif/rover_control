#!/usr/bin/env python


import rospy
from std_msgs.msg import String
import serial


BAUDRATE = 9600


def listener():
    global serial_port
    
    serial_connect(auto_select = True)

    rospy.init_node('rover', anonymous=True)
    rospy.Subscriber("rover_control", String, move_callback)
    rospy.spin()


def move_callback(message):
    serial_port.write(message.data.encode('utf-8'))
    rospy.loginfo(message.data)


def serial_connect(auto_select = False):
    global serial_port
    if auto_select:
        for usb_type in ['ttyUSB', 'ttyACM']:
            for i in range(4):
                try:
                    serial_port = serial.Serial('/dev/' + usb_type + str(i), BAUDRATE, timeout=0.01)
                    print("Connected to Arduino on port /dev/" + usb_type + str(i))
                    return
                except serial.SerialException:
                    print("Failed to connect to Arduino on port /dev/" + usb_type + str(i))
                    continue
    input_port = input("Enter the port of the Arduino: ")
    serial_port = serial.Serial(input_port, BAUDRATE)


def main():
    listener()


if __name__ == '__main__':
    main()
