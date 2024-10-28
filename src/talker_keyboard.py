#!/usr/bin/env python3
"""
ROS Keyboard Control Node

This script implements a ROS node that captures keyboard inputs and publishes
corresponding control commands for a rover. It uses the pynput library to
capture keyboard events and publishes them to the 'rover_control' topic.

The node will publish:
- The character pressed as a command when a key is pressed
- A '-' character when a key is released
- Exit when ESC key is pressed

Dependencies:
    - rospy
    - std_msgs
    - pynput

Author: Mollah Md Saif
"""

import rospy
from std_msgs.msg import String
from pynput import keyboard
from typing import Optional, Union

class KeyboardController:
    """
    A class to handle keyboard input and publish rover control commands.

    This class manages keyboard event capturing and publishes corresponding
    commands to control a rover through ROS.

    Attributes:
        publisher (rospy.Publisher): Publisher for rover control commands
        listener (keyboard.Listener): Keyboard event listener
    """

    def __init__(self):
        """Initialize the keyboard controller."""
        self.publisher: Optional[rospy.Publisher] = None
        self.listener: Optional[keyboard.Listener] = None
        self.rate: Optional[rospy.Rate] = None

    def initialize_node(self) -> None:
        """
        Initialize ROS node and set up publisher.

        This method:
        1. Initializes the ROS node named 'base_station'
        2. Creates a publisher for the 'rover_control' topic
        3. Sets up the publishing rate
        """
        rospy.init_node('base_station', anonymous=True)
        self.publisher = rospy.Publisher('rover_control', String, queue_size=10)
        self.rate = rospy.Rate(100)  # 100Hz
        rospy.loginfo("Keyboard controller node initialized")

    def publish_command(self, command: str) -> None:
        """
        Publish a command to the rover_control topic.

        Args:
            command (str): The command string to publish
        """
        rospy.loginfo(command)
        self.publisher.publish(command)

    def on_press(self, key: Union[keyboard.Key, keyboard.KeyCode]) -> None:
        """
        Handle key press events.

        This method is called whenever a key is pressed. It attempts to
        publish the character associated with the key.

        Args:
            key (Union[keyboard.Key, keyboard.KeyCode]): The key that was pressed
        """
        try:
            if hasattr(key, 'char') and key.char:
                self.publish_command(key.char)
        except AttributeError as e:
            rospy.logdebug(f"Attribute error in key press handler: {e}")

    def on_release(self, key: Union[keyboard.Key, keyboard.KeyCode]) -> Optional[bool]:
        """
        Handle key release events.

        This method is called whenever a key is released. It publishes a '-'
        character for all keys except ESC, which terminates the listener.

        Args:
            key (Union[keyboard.Key, keyboard.KeyCode]): The key that was released

        Returns:
            Optional[bool]: False if ESC is pressed (to stop listener), None otherwise
        """
        if key == keyboard.Key.esc:
            rospy.loginfo("ESC pressed - terminating keyboard listener")
            return False
        
        self.publish_command('-')
        return None

    def setup_keyboard_listener(self) -> None:
        """
        Set up the keyboard event listener.

        Creates and starts a keyboard listener that will call the appropriate
        callback methods for key press and release events.
        """
        self.listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release
        )
        self.listener.start()

    def run(self) -> None:
        """
        Main execution method.

        This method:
        1. Initializes the ROS node
        2. Sets up the keyboard listener
        3. Keeps the node running until shutdown
        """
        self.initialize_node()
        
        while not rospy.is_shutdown():
            try:
                self.setup_keyboard_listener()
                self.listener.join()  # Wait for the listener to exit
                
                # If we get here, the listener has stopped (ESC pressed)
                # Break the loop to shut down gracefully
                break
                
            except Exception as e:
                rospy.logerr(f"Error in keyboard listener: {e}")
                # Brief pause before attempting to restart the listener
                rospy.sleep(1.0)
            
            if self.rate:
                self.rate.sleep()

def main():
    """
    Main entry point for the script.
    
    Sets up the keyboard controller and handles the main ROS exception
    for clean shutdown.
    """
    try:
        controller = KeyboardController()
        controller.run()
    except rospy.ROSInterruptException as e:
        rospy.loginfo(f"Node interrupted: {e}")
    finally:
        rospy.loginfo("Keyboard controller node shutdown")

if __name__ == '__main__':
    main()