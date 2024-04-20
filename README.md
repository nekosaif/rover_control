# rover_control

# Joystick Setup and Testing in ROS

This guide outlines the process of installing, configuring, and testing a joystick in a ROS (Robot Operating System) environment.

## Installation

1. Install the ROS package for joystick control:
    ```
    sudo apt-get install ros-<DISTRO>-joy
    ```
   Replace `<DISTRO>` with your ROS distribution name.

2. Identify the joystick number by listing the input devices:
    ```
    ls /dev/input/
    ```

3. Test the joystick to ensure it's functioning properly:
    ```
    sudo jstest /dev/input/js<>
    ```

4. Provide permissions to access the joystick device:
    ```
    sudo chmod a+rw /dev/input/js<>
    ```

## Running

1. Set the joystick device parameter for ROS:
    ```
    rosparam set joy_node/dev "/dev/input/js<>"
    ```
   Replace `<>` with the appropriate joystick number.

2. Run the joy node to start reading data from the joystick:
    ```
    rosrun joy joy_node
    ```
   Optionally, you can specify a deadzone value to filter out small movements:
    ```
    rosrun joy joy_node _deadzone:=0.001
    ```

## Testing

1. Echo the joy topic to see the joystick data being published:
    ```
    rostopic echo joy
    ```

