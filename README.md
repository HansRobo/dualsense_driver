# dualsense driver
## requirements
~~~bash
sudo apt install libhidapi-dev
pip3 install pydualsense
~~~
## usage
### Connect DualSense
Supported connection types are USB only.
### Permissions
~~~bash
sudo chmod 606 /dev/hidraw*
~~~
### Run DualSense Driver
~~~bash
rosrun dualsense_driver dualsense_driver.py
~~~
