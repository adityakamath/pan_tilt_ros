# pan_tilt_ros
ROS 2 C++ package for controlling a pan-tilt mechanism using [Joy messages](https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Joy.html). The pan-tilt mechanism is built using two [Waveshare ST3215](https://www.waveshare.com/wiki/ST3215_Servo) serial bus servo motors. It is also compatible with the [Feetech STS3215](https://www.feetechrc.com/en/2020-05-13_56655.html) and [Waveshare ST3215-HS](https://www.waveshare.com/st3215-hs-servo-motor.htm) motors. 

The mechanism currently holds a [Luxonis OAK-D](https://shop.luxonis.com/products/oak-d) stereo camera but can be modified to attach any sensor device. This package is solely for controlling the pan-tilt mechanism and does not contain any nodes for the sensor payload. 

## Implementation details

* ```pan_tilt_cmd```: This executable is generated using the ```pan_tilt_cmd_node.cpp``` source file and converts [Joy](https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Joy.html) messages to [JointState](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/JointState.html) messages using up to two joystick axes each for the pan and tilt movements. In this case, the pan motion uses the L2 and R2 controls for left and right motion and the tilt motion is controlled using the Up/Down buttons on the D-Pad. The configuration allows up to 2 axes per joint, which are then mixed in the source code. 
* ```pan_tilt_ctrl```: This executable is generated using the ```pan_tilt_ctrl_node.cpp``` source file and subscribes to JointState messages which is used to control the motors, and Joy messages to determine the status of the emergency stop. When the emergency stop button is pressed, the torque is disabled on both motors, allowing the user to move the motors by hand. When the button is pressed again, the motors return to the position specified by the pan and tilt axes - which can be defined in the configuration file. This node uses the [SCServo_Linux library](https://github.com/adityakamath/SCServo_Linux) to drive the motors in the closed-loop servo mode. Additionally, this node also publishes measured JointState values and [DiagnosticArray](https://docs.ros2.org/foxy/api/diagnostic_msgs/msg/DiagnosticArray.html) messages (PWM, motion - true/false, temperature, voltage and current) for each motor.
* ```pan_tilt_launch.py```: This is the launch file that launches ```pan_tilt_cmd``` and ```pan_tilt_ctrl``` nodes, and loads their respective parameters using the ```config/cmd_config.yaml``` and ```config/ctrl_config.yaml``` files.

## Parameters
The following parameters and their default values are defined in the ```pan_tilt_cmd``` node. In this package, the default values are overwritten by the ```config/cmd_config.yaml``` file.

* ```joint_names```: Joint names for the pan and tilt joints (Default: ```[joint_pan, joint_tilt]```)
* ```joint_inv```: Invert joint direction (Default: ```[false, false]```)
* ```joy_axes0```: Joystick axes mapping for index 0 of ```joint_names``` (Default: ```[2, 5]```)
* ```joy_axes1```: Joystick axes mapping for index 1 of ```joint_names``` (Default: ```[7]```)
* ```mid_pos```: Middle (origin) step values for the pan and tilt motors (Default: ```[2048, 2048]```)
* ```min_pos```: Lower limit step values for the pan and tilt motors (Default: ```[1600, 1600]```)
* ```max_pos```: Upper limit step values for the pan and tilt motors (Default: ```[2816, 2816]```)

The following parameters and their default values are defined in the ```pan_tilt_ctrl``` node. In this package, the default values are overwritten by the ```config/ctrl_config.yaml``` file.

* ```joint_ids```: Motor IDs for each joint (Default: ```[1, 2]```)
* ```stop_button```: Button mapping for the emergency stop functionality (Default: ```4```)
* ```speed```: Speed of the motors in steps per second ranging from -4500 to 4500 (```Default: 4500```)
* ```acceleration```: Acceleration of the motors ranging from 0 to 255 (Default: ```255```)
* ```usb_port```: USB port name (Default: ```/dev/ttyACM0```)
* ```baud_rate```: Baud rate of the motor driver (Default: ```1000000```)


## How to use

* Clone this repository in your ROS 2 workspace
* Navigate to the repository and run: ```git submodule init && git submodule update```
* Check the ```config.yaml``` file in the config directory, and make any necessary changes
* If needed, configure the mid-point of the motors using examples in ```include/SCServoLinux``` or the [Feetech Debugger](https://www.feetechrc.com/Data/feetechrc/upload/file/20201127/start%20%20tutorial201015.pdf)
* Build the package and run the launch file: ```ros2 launch pan_tilt_ros pan_tilt_launch.py```
* On a separate terminal, launch your joystick node. In this case, the [joystick_drivers](https://github.com/adityakamath/joystick_drivers) package is used.

