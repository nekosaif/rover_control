#!/usr/bin/env python
# license removed for brevity

import rospy
from std_msgs.msg import String
from time import sleep
from pynput import keyboard


def talker():
    global publisher
    publisher = rospy.Publisher('rover_control', String, queue_size=10)
    rospy.init_node('base_station', anonymous=True)

    rate = rospy.Rate(100)

    while not rospy.is_shutdown():
        with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
            listener.join()
        rate.sleep()


def move(instruction_key):
    rospy.loginfo(instruction_key)
    publisher.publish(instruction_key)


def on_press(key):
    try:
        move(key.char)
    except AttributeError:
        pass

def on_release(key):
    if key == keyboard.Key.esc:
        return False
    move('-')


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass