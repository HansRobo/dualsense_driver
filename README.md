# dualsense driver

## Requirements

```bash
sudo apt install libhidapi-dev
pip3 install pydualsense
```

## Usage

### Connect DualSense

Supported connection types are USB only.

### Permissions

If you set the permission only this time, execute the following command:

```bash
sudo chmod 606 /dev/hidraw*  # or specify the device
```

On the other hand, if you would like to do it permanently, update the udev rules with the following commands:

```bash
sudo cp config/99-dualsense.rules /etc/udev/rules.d/
sudo service udev restart
```

### Run DualSense Driver

```bash
rosrun dualsense_driver dualsense_driver.py
```
