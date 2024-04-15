
## Connecting PS4 controller to Raspberry Pi via Bluetooth
https://salamwaddah.com/blog/connecting-ps4-controller-to-raspberry-pi-via-bluetooth



### Pairing a device

When using your controller on a device for the first time, you must pair it.

1. Make sure the light bar on your controller is off. If the light bar is on, press and hold the PS button until it turns off. If a USB cable is connected to your controller, disconnect it.
2. While pressing and holding the SHARE button, press and hold the PS button until the light bar flashes.
3. Enable Bluetooth® on your device, and then select your controller from the list of Bluetooth® devices.When pairing is complete, the light bar turns a solid color. 

https://controller.dl.playstation.net/controller/lang/en/


### Test joystic 

sudo apt install jstest-gtk

start the app jstest-gtk

## ros::Publisher  cmd_vel

https://github.com/ros-teleop/teleop_twist_keyboard/tree/master




## ros::Subscriber cmd_vel

referance 
https://github.com/MichalDobis/osm_planner/blob/cdb1368552eb4ac6a71c85660eeaf7a220ed9582/src/navigation_example.cpp#L79 


https://github.com/ros-teleop/teleop_twist_keyboard/tree/master


### teleop_twist_joy

http://wiki.ros.org/teleop_twist_joy

    rosrun teleop_twist_joy teleop_node

**joystick_drivers**

https://github.com/ros-drivers/joystick_drivers/tree/main

    rosrun joy joy_node  
