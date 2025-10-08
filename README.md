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
ros2 run dualsense_driver dualsense_driver.py
```

### Set ramble motor power

Set 0~255 motor power to `left_motor` / `right_motor` ROS 2 parameters.

```bash
ros2 param set /dualsense_driver left_motor 255
```

### Published topics

- `/joy` (`sensor_msgs/msg/Joy`): sticks, triggers, and buttons in the classic joystick layout.
- `/dualsense/imu` (`sensor_msgs/msg/Imu`): raw gyro/accelerometer readings scaled by `gyro_scale` / `accelerometer_scale`.
- `/dualsense/battery` (`sensor_msgs/msg/BatteryState`): battery percentage and charging state reported by the controller.
- `/dualsense/touch` (`sensor_msgs/msg/Joy`): touchpad points (`axes = [x0, y0, id0, x1, y1, id1]`, `buttons = [active0, active1]`).
- `/dualsense/button/mic` (`std_msgs/msg/Bool`): microphone button edge events.
- `/dualsense/button/touch` (`std_msgs/msg/Bool`): touchpad button edge events.

### Subscribed topics

- `/dualsense/indicator` (`std_msgs/msg/Bool`): toggles between `indicator_on_color` / `indicator_off_color`.
- `/dualsense/light/color` (`std_msgs/msg/ColorRGBA`): sets indicator color (values 0.0–1.0 or 0–255).
- `/dualsense/light/player` (`std_msgs/msg/UInt8`): selects player LED (0 = ALL, 1–4 = PLAYER_1..4).
- `/dualsense/light/brightness` (`std_msgs/msg/String`): sets LED brightness (`low`, `medium`, `high`).
- `/dualsense/microphone/mute` (`std_msgs/msg/Bool`): mutes/unmutes the microphone.
- `/dualsense/microphone/led` (`std_msgs/msg/Bool`): toggles the microphone LED.
- `/dualsense/rumble/left` (`std_msgs/msg/Float32`): left motor intensity (0.0–1.0).
- `/dualsense/rumble/right` (`std_msgs/msg/Float32`): right motor intensity (0.0–1.0).
- `/dualsense/trigger_mode_left` (`std_msgs/msg/String`): updates left trigger mode (e.g. `Off`, `Rigid`, `Pulse_A`).
- `/dualsense/trigger_mode_right` (`std_msgs/msg/String`): updates right trigger mode.

### Parameters

- `left_motor` / `right_motor` (int, 0–255): rumble motor power (kept for compatibility).
- `indicator_topic` (string): topic used to toggle the indicator light.
- `indicator_on_color` / `indicator_off_color` (int[3]): RGB colors for indicator toggle.
- `publish_imu`, `publish_battery`, `publish_touchpad`, `publish_button_events` (bool): enable/disable individual publishers.
- `accelerometer_scale`, `gyro_scale` (float): scaling factors applied to IMU data.
- `frame_id` (string): frame id stamped on IMU messages.
- `left_trigger_mode`, `right_trigger_mode` (string): initial trigger modes (same strings as topic interface).
- `light_brightness` (string), `light_player_id` (string): initial LED settings (`light_player_id` accepts `PLAYER_1`..`PLAYER_4`, `ALL`).
