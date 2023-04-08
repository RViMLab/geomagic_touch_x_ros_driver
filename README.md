# Geomagic Touch X ROS Driver

# Introduction

Node for using the Geomagic Touch X connected via ethernet with the Robot Operating System (ROS).
Currently tested and working with ROS Noetic (Ubuntu 20.04).

# Install

This assumes you have installed the driver software provided by 3D Systems for the device in the default location.

1. `$ cd catkin_ws/src/`
2. `$ git clone git@github.com:RViMLab/geomagic_touch_x_ros_driver.git geomagic_touch_x_ros`
3. `$ catkin build -s`
4. Plug in the device to your computer via Ethernet.
5. Setup a new connection:
  * Name: Haptic TouchX
  * IPv4 Method: "Link-Local Only"
  * IPv6 Method: "Link-Local Only"

# Usage

This assumes you have installed the driver software provided by 3D Systems for the device in the default location.

## Pair device

Run this each time you use the device.

1. Plug in the device to your computer via Ethernet.
2. Ensure the correct wired connection is selected.
3. `$ cd /path/to/geomagic_touch_x_ros_driver`
4. `$ bash pair.sh`
5. Click "Rescan for Devices", this should ensure we can find the device.
6. Click "Pairing", and just after click the pairing button on the device.
7. The device should successfully pair.
8. Click "Apply" and then "Ok".

If this doesn't work, then see the official documentation provided with your device.

## Calibrate the device

Run this each time you use the device after the device has been paired (see previous section).

1. `$ cd /path/to/geomagic_touch_x_ros_driver`
2. `$ bash calibrate.sh`
3. Click the "Select" tab.
4. Then click the next button (right arrow) at the bottom of the dialog box.
5. Move the stylus in each axis (X, Y, Z) to properly calibrate the device.
6. Once each axis is calibrated, its icon will turn green.
7. Step through each other test, and then once each is complete, click the cross to exit the diagonostic setup.

If this doesn't work, then see the official documentation provided with your device.

# `geomagic_touch_x_node`

This is the main ROS node you need to launch to set/get the state of the haptic device.

## Parameters

* `~device_name` (string): the name of the device, this can be set during pairing (see above)

## Subscribed Topics

* `~cmd_force` (`geometry_msgs/Wrench`): the force that should be commanded at the device end-effector. Note, only the linear part is used for the Touch X device.

## Published Topics

* `~twist` (`geometry_msgs/Twist`): the linear and angular velocity of the device end-effector.
* `~joint_states` (`sensor_msgs/JointState`): the joint states of the device.

## Transform (`tf2`)

The transform of the device end-effector is broadcast using the `tf2` library.
The base frame is `touch_x_base` and the child frame is `touch_x_ee`.
