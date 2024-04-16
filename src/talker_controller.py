#!/usr/bin/env python3


import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Joy

'''
Installation:
1) "sudo apt-get install ros-<DISTRO>-joy"
2) Check the Joystick Number "js<>": ls /dev/input/"
3) Test the Joystick: "sudo jstest /dev/input/js<>"
4) Give Permission: "sudo chmod a+rw /dev/input/js<>"

Running:
1) Set the Joystick: "rosparam set joy_node/dev "/dev/input/js<>""
2) Joy Node Run: "rosrun joy joy_node" / Optional Deadzone: "rosrun joy joy_node _deadzone:=0.001"

Testing:
1) Run Test Node: "rostopic echo joy"
'''


BUTTONS = {
    0: 'f',     # A
    1: 'm',     # B
    2: 'n',     # X
    3: 'r',     # Y
    4: 't',     # LB
    5: 'y',     # RB
    6: 'k',     # Back
    7: 'l',     # Start
    8: '',     # Logo
    9: '.',     # LS
    10: '/',    # RS
}


AXES = {
    0: [2, [], ['v', 'b']],                 # DL DR
    1: [2, [], ['o', 'p']],                 # DU DD
    2: [1, [[1, -1]], ['g']],               # LT
    3: [2, [[0, 1], [0, -1]], ['a', 'd']],  # RSL RSR
    4: [2, [[0, 1], [0, -1]], ['w', 's']],  # RSU RSD
    5: [1, [[1, -1]], ['h']],               # RT
    6: [2, [[0, 1], [0, -1]], ['', '']],    # LSL LSR
    7: [2, [[0, 1], [0, -1]], ['', '']],    # LSU LSD
}


is_pressed_button = [False for i in range(11)]
is_pressed_axes = [[0, 0] for i in range(8)]


def talker():
    global control_publisher, joy_subscriber, sensor_msgs_joy
    
    control_publisher = rospy.Publisher('rover_control', String, queue_size=10)
    joy_subscriber = rospy.Subscriber('joy', Joy, joy_callback)

    rospy.init_node('base_station', anonymous=True)

    sensor_msgs_joy = None

    rate = rospy.Rate(100)

    while not rospy.is_shutdown():
        move()
        rate.sleep()


def joy_callback(msgs):
    global sensor_msgs_joy
    sensor_msgs_joy = msgs


def move():
    global control_publisher

    buttons = tuple([i for i in range(11)]) if sensor_msgs_joy == None else sensor_msgs_joy.buttons
    axes = tuple([i for i in range(8)]) if sensor_msgs_joy == None else sensor_msgs_joy.axes

    for i_button, value_button in enumerate(buttons):
        if value_button > 0 and not is_pressed_button[i_button]:
            key = str(BUTTONS[i_button]) + '255'
            is_pressed_button[i_button] = True
            control_publisher.publish(key)
            rospy.loginfo('{} Published'.format(key))
        elif value_button == 0 and is_pressed_button[i_button]:
            key = str(BUTTONS[i_button]).capitalize() + '0'
            is_pressed_button[i_button] = False
            control_publisher.publish(key)
            rospy.loginfo('{} Published'.format(key))

    for i_axis, value_axis in enumerate(axes):
        if AXES[i_axis][0] == 1:
            value = int(((-1)*value_axis+1)*127.5)
            if value != 0 and is_pressed_axes[i_axis][0] != value:
                key = str(AXES[i_axis][2][0]) + str(value)
                is_pressed_axes[i_axis][0] = value
                control_publisher.publish(key)
                rospy.loginfo('{} Published'.format(key))
            elif value == 0 and is_pressed_axes[i_axis][0] != 0:
                key = str(AXES[i_axis][2][0]).capitalize() + '0'
                is_pressed_axes[i_axis][0] = 0
                control_publisher.publish(key)
                rospy.loginfo('{} Published'.format(key))
        elif AXES[i_axis][0] == 2:
            value = int(abs(value_axis * 255)) if AXES[i_axis][1] else (0 if value_axis == 0 else 255)
            if value == 0 and (is_pressed_axes[i_axis][0] != 0 or is_pressed_axes[i_axis][1] != 0):
                key = str(AXES[i_axis][2][0]).capitalize() + '0'
                is_pressed_axes[i_axis][0] = 0
                control_publisher.publish(key)
                rospy.loginfo('{} Published'.format(key))
                key = str(AXES[i_axis][2][1]).capitalize() + '0'
                is_pressed_axes[i_axis][1] = 0
                control_publisher.publish(key)
                rospy.loginfo('{} Published'.format(key))
            elif value != 0 and (is_pressed_axes[i_axis][0] != value or is_pressed_axes[i_axis][1] != value):
                #alt_key = str(AXES[i_axis][2][1 if value_axis > 0 else 0]).capitalize() + '0'
                #is_pressed_axes[i_axis][1 if value_axis > 0 else 0] = 0
                #control_publisher.publish(alt_key)
                #rospy.loginfo('{} Published'.format(alt_key))
                key = str(AXES[i_axis][2][0 if value_axis > 0 else 1]) + str(value)
                is_pressed_axes[i_axis][0 if value_axis > 0 else 1] = value
                control_publisher.publish(key)
                rospy.loginfo('{} Published'.format(key))


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass