#!/usr/bin/env python3
"""
ROS Joystick Controller Node

This script implements a ROS node that interfaces with a joystick/gamepad controller
and publishes control commands for a rover. It maps joystick buttons and axes to
specific control commands.

Installation Requirements:
    1. Install ROS joy package:
       $ sudo apt-get install ros-<DISTRO>-joy
    2. Check joystick number: 
       $ ls /dev/input/
    3. Test joystick:
       $ sudo jstest /dev/input/js<number>
    4. Set permissions:
       $ sudo chmod a+rw /dev/input/js<number>

Usage:
    1. Set joystick device:
       $ rosparam set joy_node/dev "/dev/input/js<number>"
    2. Run joy node:
       $ rosrun joy joy_node
       Optional: Set deadzone:
       $ rosrun joy joy_node _deadzone:=0.001
    3. Test node:
       $ rostopic echo joy

Author: Mollah Md Saif
"""

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Joy
from typing import Dict, List, Tuple, Optional, Union

class JoystickController:
    """
    A class to handle joystick input and publish rover control commands.

    This class manages the mapping of joystick inputs to rover commands and handles
    the publishing of these commands to the ROS network.

    Attributes:
        BUTTONS (Dict[int, str]): Mapping of button indices to command characters
        AXES (Dict[int, List]): Mapping of axes to their properties and commands
    """

    # Button mapping configuration
    BUTTONS: Dict[int, str] = {
        0: 'f',     # A
        1: 'm',     # B
        2: 'n',     # X
        3: 'r',     # Y
        4: 't',     # LB
        5: 'y',     # RB
        6: 'k',     # Back
        7: 'l',     # Start
        8: '',      # Logo
        9: '.',     # LS
        10: '/',    # RS
    }

    # Axes mapping configuration
    AXES: Dict[int, List] = {
        0: [2, [], ['v', 'b']],                 # DL DR
        1: [2, [], ['o', 'p']],                 # DU DD
        2: [1, [[1, -1]], ['g']],               # LT
        3: [2, [[0, 1], [0, -1]], ['a', 'd']],  # RSL RSR
        4: [2, [[0, 1], [0, -1]], ['w', 's']],  # RSU RSD
        5: [1, [[1, -1]], ['h']],               # RT
        6: [2, [[0, 1], [0, -1]], ['', '']],    # LSL LSR
        7: [2, [[0, 1], [0, -1]], ['', '']],    # LSU LSD
    }

    def __init__(self):
        """Initialize the joystick controller node."""
        self.control_publisher: Optional[rospy.Publisher] = None
        self.joy_subscriber: Optional[rospy.Subscriber] = None
        self.sensor_msgs_joy: Optional[Joy] = None
        
        # Track button and axes states
        self.is_pressed_button = [False for _ in range(11)]
        self.is_pressed_axes = [[0, 0] for _ in range(8)]

    def initialize_node(self) -> None:
        """Initialize ROS node and set up publisher/subscriber."""
        rospy.init_node('base_station', anonymous=True)
        self.control_publisher = rospy.Publisher('rover_control', String, queue_size=10)
        self.joy_subscriber = rospy.Subscriber('joy', Joy, self.joy_callback)
        rospy.loginfo("Base station node initialized")

    def joy_callback(self, msgs: Joy) -> None:
        """
        Callback function for joystick messages.

        Args:
            msgs (Joy): The joystick message containing button and axes states
        """
        self.sensor_msgs_joy = msgs

    def process_buttons(self, buttons: Tuple[int, ...]) -> None:
        """
        Process button inputs and publish corresponding commands.

        Args:
            buttons (Tuple[int, ...]): Tuple of button states
        """
        for i_button, value_button in enumerate(buttons):
            if not self.BUTTONS[i_button]:  # Skip empty mappings
                continue
                
            if value_button > 0 and not self.is_pressed_button[i_button]:
                key = f"{self.BUTTONS[i_button]}255"
                self.is_pressed_button[i_button] = True
                self._publish_command(key)
            elif value_button == 0 and self.is_pressed_button[i_button]:
                key = f"{self.BUTTONS[i_button].upper()}0"
                self.is_pressed_button[i_button] = False
                self._publish_command(key)

    def process_axes(self, axes: Tuple[float, ...]) -> None:
        """
        Process axes inputs and publish corresponding commands.

        Args:
            axes (Tuple[float, ...]): Tuple of axes values
        """
        for i_axis, value_axis in enumerate(axes):
            if self.AXES[i_axis][0] == 1:
                self._process_single_axis(i_axis, value_axis)
            elif self.AXES[i_axis][0] == 2:
                self._process_dual_axis(i_axis, value_axis)

    def _process_single_axis(self, i_axis: int, value_axis: float) -> None:
        """
        Process a single-direction axis input.

        Args:
            i_axis (int): Index of the axis
            value_axis (float): Value of the axis
        """
        value = int(((-1) * value_axis + 1) * 127.5)
        if value != 0 and self.is_pressed_axes[i_axis][0] != value:
            key = f"{self.AXES[i_axis][2][0]}{value}"
            self.is_pressed_axes[i_axis][0] = value
            self._publish_command(key)
        elif value == 0 and self.is_pressed_axes[i_axis][0] != 0:
            key = f"{self.AXES[i_axis][2][0].upper()}0"
            self.is_pressed_axes[i_axis][0] = 0
            self._publish_command(key)

    def _process_dual_axis(self, i_axis: int, value_axis: float) -> None:
        """
        Process a dual-direction axis input.

        Args:
            i_axis (int): Index of the axis
            value_axis (float): Value of the axis
        """
        value = int(abs(value_axis * 255)) if self.AXES[i_axis][1] else (0 if value_axis == 0 else 255)
        
        if value == 0 and (self.is_pressed_axes[i_axis][0] != 0 or self.is_pressed_axes[i_axis][1] != 0):
            # Reset both directions
            for i in range(2):
                key = f"{self.AXES[i_axis][2][i].upper()}0"
                self.is_pressed_axes[i_axis][i] = 0
                self._publish_command(key)
        elif value != 0 and (self.is_pressed_axes[i_axis][0] != value or self.is_pressed_axes[i_axis][1] != value):
            direction = 0 if value_axis > 0 else 1
            key = f"{self.AXES[i_axis][2][direction]}{value}"
            self.is_pressed_axes[i_axis][direction] = value
            self._publish_command(key)

    def _publish_command(self, key: str) -> None:
        """
        Publish a command to the rover_control topic.

        Args:
            key (str): The command string to publish
        """
        self.control_publisher.publish(key)
        rospy.loginfo(f'{key} Published')

    def run(self) -> None:
        """
        Main execution method.

        Runs the control loop at 100Hz, processing joystick inputs and
        publishing commands.
        """
        self.initialize_node()
        rate = rospy.Rate(100)  # 100Hz

        while not rospy.is_shutdown():
            if self.sensor_msgs_joy is None:
                buttons = tuple(0 for _ in range(11))
                axes = tuple(0 for _ in range(8))
            else:
                buttons = self.sensor_msgs_joy.buttons
                axes = self.sensor_msgs_joy.axes

            self.process_buttons(buttons)
            self.process_axes(axes)
            rate.sleep()

def main():
    """Main entry point for the script."""
    try:
        controller = JoystickController()
        controller.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node interrupted")

if __name__ == '__main__':
    main()