#!/usr/bin/env python3

from __future__ import annotations

import math
from typing import Dict, Optional, Sequence, Tuple

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from sensor_msgs.msg import BatteryState as BatteryStateMsg
from sensor_msgs.msg import Imu, Joy
from std_msgs.msg import Bool, ColorRGBA, Float32, String, UInt8

from pydualsense import pydualsense
from pydualsense.enums import (
    BatteryState,
    Brightness,
    PlayerID,
    TriggerModes,
)


class DualSenseDriver(Node):
    def __init__(self) -> None:
        super().__init__('dualsense_driver')

        self.dualsense = pydualsense()
        self._connect_dualsense()

        self.frame_id = str(self.declare_parameter('frame_id', 'dualsense').value)
        self.publish_imu = bool(self.declare_parameter('publish_imu', True).value)
        self.publish_battery = bool(self.declare_parameter('publish_battery', True).value)
        self.publish_touchpad = bool(self.declare_parameter('publish_touchpad', True).value)
        self.publish_button_events = bool(
            self.declare_parameter('publish_button_events', True).value)

        self.accel_scale = float(self.declare_parameter('accelerometer_scale', 1.0).value)
        self.gyro_scale = float(self.declare_parameter('gyro_scale', 1.0).value)

        self.left_motor_power = self._declare_motor_parameter('left_motor', 0)
        self.right_motor_power = self._declare_motor_parameter('right_motor', 0)

        self.indicator_topic = str(
            self.declare_parameter('indicator_topic', '/dualsense/indicator').value)
        self.indicator_on_color = self._declare_color_param(
            'indicator_on_color', [0, 100, 255])
        self.indicator_off_color = self._declare_color_param(
            'indicator_off_color', [255, 40, 0])

        self.left_trigger_mode = str(
            self.declare_parameter('left_trigger_mode', 'Off').value)
        self.right_trigger_mode = str(
            self.declare_parameter('right_trigger_mode', 'Off').value)

        self.light_brightness = str(
            self.declare_parameter('light_brightness', 'medium').value)
        self.light_player_id = str(
            self.declare_parameter('light_player_id', 'PLAYER_1').value)

        self._trigger_modes_map = {
            name.upper(): mode for name, mode in TriggerModes.__members__.items()
        }

        self.joy_pub = self.create_publisher(Joy, '/joy', 10)
        self.imu_pub = self.create_publisher(Imu, '/dualsense/imu', 10)
        self.battery_pub = self.create_publisher(BatteryStateMsg, '/dualsense/battery', 10)
        self.touch_pub = self.create_publisher(Joy, '/dualsense/touch', 10)
        self.mic_button_pub = self.create_publisher(Bool, '/dualsense/button/mic', 10)
        self.touch_button_pub = self.create_publisher(Bool, '/dualsense/button/touch', 10)

        self.timer = self.create_timer(1.0 / 60.0, self.update)

        self._set_light_fn = self._resolve_light_api()
        self._indicator_state: Optional[bool] = None
        self._indicator_subscription = None
        self._create_indicator_subscription(self.indicator_topic)
        self._update_indicator_light(False, force=True)

        self._apply_motor_power('left', self.left_motor_power)
        self._apply_motor_power('right', self.right_motor_power)
        self._apply_trigger_mode('left', self.left_trigger_mode, silent=True)
        self._apply_trigger_mode('right', self.right_trigger_mode, silent=True)
        self._apply_light_brightness(self.light_brightness, silent=True)
        self._apply_player_id(self.light_player_id, silent=True)

        self._last_mic_button: Optional[bool] = None
        self._last_touch_button: Optional[bool] = None

        self.add_on_set_parameters_callback(self.on_set_parameter_callback)

        self._create_command_subscriptions()

    def _connect_dualsense(self) -> None:
        try:
            self.dualsense.init()
        except Exception as exc:  # pragma: no cover - hardware dependent
            self.get_logger().fatal(f'Failed to initialize DualSense: {exc}')
            raise

    def _declare_motor_parameter(self, name: str, default: int) -> int:
        value = int(self.declare_parameter(name, default).value)
        return max(0, min(255, value))

    def _declare_color_param(
        self,
        name: str,
        default: Sequence[int],
    ) -> Tuple[int, int, int]:
        value = self.declare_parameter(name, list(default)).value
        return self._normalize_color(value, tuple(default), name, warn=False)

    def _create_command_subscriptions(self) -> None:
        self.create_subscription(ColorRGBA, '/dualsense/light/color', self._on_light_color, 10)
        self.create_subscription(UInt8, '/dualsense/light/player', self._on_player_id_cmd, 10)
        self.create_subscription(String, '/dualsense/light/brightness', self._on_light_brightness_cmd, 10)
        self.create_subscription(Bool, '/dualsense/microphone/mute', self._on_microphone_mute, 10)
        self.create_subscription(Bool, '/dualsense/microphone/led', self._on_microphone_led, 10)
        self.create_subscription(Float32, '/dualsense/rumble/left', self._on_left_rumble, 10)
        self.create_subscription(Float32, '/dualsense/rumble/right', self._on_right_rumble, 10)
        self.create_subscription(String, '/dualsense/trigger_mode_left', self._on_trigger_mode_left, 10)
        self.create_subscription(String, '/dualsense/trigger_mode_right', self._on_trigger_mode_right, 10)

    def on_set_parameter_callback(self, parameter_list):
        for param in parameter_list:
            if param.name == 'right_motor':
                self.right_motor_power = self._apply_motor_power('right', param.value)
            elif param.name == 'left_motor':
                self.left_motor_power = self._apply_motor_power('left', param.value)
            elif param.name == 'indicator_on_color':
                new_color = self._normalize_color(param.value, self.indicator_on_color, param.name)
                if new_color != self.indicator_on_color:
                    self.indicator_on_color = new_color
                    self._update_indicator_light(self._current_indicator_state(), force=True)
            elif param.name == 'indicator_off_color':
                new_color = self._normalize_color(param.value, self.indicator_off_color, param.name)
                if new_color != self.indicator_off_color:
                    self.indicator_off_color = new_color
                    self._update_indicator_light(self._current_indicator_state(), force=True)
            elif param.name == 'indicator_topic':
                new_topic = str(param.value).strip()
                if not new_topic:
                    self.get_logger().warn('indicator_topic cannot be empty.')
                    continue
                if new_topic != self.indicator_topic:
                    self.indicator_topic = new_topic
                    self._create_indicator_subscription(new_topic)
                    self.get_logger().info(f'Indicator subscription topic switched to {new_topic}.')
            elif param.name == 'publish_imu':
                self.publish_imu = bool(param.value)
            elif param.name == 'publish_battery':
                self.publish_battery = bool(param.value)
            elif param.name == 'publish_touchpad':
                self.publish_touchpad = bool(param.value)
            elif param.name == 'publish_button_events':
                self.publish_button_events = bool(param.value)
            elif param.name == 'accelerometer_scale':
                self.accel_scale = float(param.value)
            elif param.name == 'gyro_scale':
                self.gyro_scale = float(param.value)
            elif param.name == 'frame_id':
                self.frame_id = str(param.value)
            elif param.name == 'left_trigger_mode':
                self.left_trigger_mode = str(param.value)
                self._apply_trigger_mode('left', self.left_trigger_mode)
            elif param.name == 'right_trigger_mode':
                self.right_trigger_mode = str(param.value)
                self._apply_trigger_mode('right', self.right_trigger_mode)
            elif param.name == 'light_brightness':
                self.light_brightness = str(param.value)
                self._apply_light_brightness(self.light_brightness)
            elif param.name == 'light_player_id':
                self.light_player_id = str(param.value)
                self._apply_player_id(self.light_player_id)
        return SetParametersResult(successful=True)

    def update(self) -> None:
        stamp = self.get_clock().now().to_msg()
        state = self.dualsense.state

        joy_msg = Joy()
        joy_msg.header.stamp = stamp
        joy_msg.axes = [
            -state.LX / 128.0,
            -state.LY / 128.0,
            -state.RX / 128.0,
            -state.RY / 128.0,
            state.L2 / 256.0,
            state.R2 / 256.0,
            0.0,
            0.0,
            0.0,
            self._axis_from_dpad(state.DpadRight, state.DpadLeft),
            self._axis_from_dpad(state.DpadUp, state.DpadDown),
        ]

        joy_msg.buttons = [
            int(state.square),
            int(state.cross),
            int(state.circle),
            int(state.triangle),
            int(state.L1),
            int(state.R1),
            int(state.L2Btn),
            int(state.R2Btn),
            int(state.share),
            int(state.options),
            int(state.R3),
            int(state.L3),
            int(state.ps),
            int(state.touchBtn),
        ]
        self.joy_pub.publish(joy_msg)

        if self.publish_touchpad:
            touch_msg = Joy()
            touch_msg.header.stamp = stamp
            touch_msg.axes = [
                float(state.trackPadTouch0.X),
                float(state.trackPadTouch0.Y),
                float(state.trackPadTouch0.ID),
                float(state.trackPadTouch1.X),
                float(state.trackPadTouch1.Y),
                float(state.trackPadTouch1.ID),
            ]
            touch_msg.buttons = [
                int(state.trackPadTouch0.isActive),
                int(state.trackPadTouch1.isActive),
            ]
            self.touch_pub.publish(touch_msg)

        if self.publish_imu:
            imu_msg = Imu()
            imu_msg.header.stamp = stamp
            imu_msg.header.frame_id = self.frame_id
            imu_msg.angular_velocity.x = state.gyro.Roll * self.gyro_scale
            imu_msg.angular_velocity.y = state.gyro.Yaw * self.gyro_scale
            imu_msg.angular_velocity.z = state.gyro.Pitch * self.gyro_scale
            imu_msg.linear_acceleration.x = state.accelerometer.X * self.accel_scale
            imu_msg.linear_acceleration.y = state.accelerometer.Y * self.accel_scale
            imu_msg.linear_acceleration.z = state.accelerometer.Z * self.accel_scale
            self.imu_pub.publish(imu_msg)

        if self.publish_battery:
            battery_msg = BatteryStateMsg()
            battery_msg.header.stamp = stamp
            battery_msg.percentage = float(self.dualsense.battery.Level) / 100.0
            battery_msg.power_supply_status = self._convert_battery_status(
                self.dualsense.battery.State)
            battery_msg.present = True
            self.battery_pub.publish(battery_msg)

        if self.publish_button_events:
            self._publish_button_event(self.mic_button_pub, state.micBtn, '_last_mic_button')
            self._publish_button_event(self.touch_button_pub, state.touchBtn, '_last_touch_button')

    def _publish_button_event(
        self,
        publisher,
        current_value: bool,
        attr_name: str,
    ) -> None:
        last_value = getattr(self, attr_name)
        if last_value == current_value:
            return
        setattr(self, attr_name, current_value)
        msg = Bool()
        msg.data = bool(current_value)
        publisher.publish(msg)

    def _axis_from_dpad(self, positive: bool, negative: bool) -> float:
        if positive and not negative:
            return 1.0
        if negative and not positive:
            return -1.0
        return 0.0

    def on_indicator_state(self, msg: Bool) -> None:
        self._update_indicator_light(msg.data)

    def _normalize_color(
        self,
        value,
        fallback: Tuple[int, int, int],
        param_name: Optional[str] = None,
        warn: bool = True,
    ) -> Tuple[int, int, int]:
        if isinstance(value, (list, tuple)) and len(value) == 3:
            try:
                rgb = tuple(max(0, min(255, int(component))) for component in value)
                return rgb  # type: ignore[return-value]
            except (TypeError, ValueError):
                pass
        if warn and param_name:
            self.get_logger().warn(f'Invalid color specification: {param_name}')
        return fallback

    def _current_indicator_state(self) -> bool:
        return self._indicator_state if self._indicator_state is not None else False

    def _create_indicator_subscription(self, topic_name: str) -> None:
        if self._indicator_subscription is not None:
            self.destroy_subscription(self._indicator_subscription)
        self._indicator_subscription = self.create_subscription(
            Bool, topic_name, self.on_indicator_state, 10)
        self._update_indicator_light(self._current_indicator_state(), force=True)

    def _resolve_light_api(self):
        light = getattr(self.dualsense, 'light', None)
        if light is None:
            self.get_logger().warn('DualSense LED control interface is not available.')
            return None

        if hasattr(light, 'setColor'):
            return light.setColor
        if hasattr(light, 'setRGB'):
            return light.setRGB
        if hasattr(light, 'setColorI'):
            return light.setColorI
        if hasattr(light, 'setColorT'):
            return lambda r, g, b: light.setColorT((r, g, b))

        self.get_logger().warn('DualSense LED color API is not available.')
        return None

    def _update_indicator_light(self, indicator_enabled: bool, force: bool = False) -> None:
        state = bool(indicator_enabled)
        if not force and self._indicator_state == state:
            return
        self._indicator_state = state
        if self._set_light_fn is None:
            return

        rgb = self.indicator_on_color if state else self.indicator_off_color
        self._apply_light(rgb)

    def _apply_light(self, rgb: Tuple[int, int, int]) -> None:
        if self._set_light_fn is None:
            return
        try:
            self._set_light_fn(*rgb)
        except Exception as exc:  # pragma: no cover - hardware dependent
            self.get_logger().warn(f'Failed to update LED color: {exc}')
            self._set_light_fn = None

    def _apply_motor_power(self, side: str, value) -> int:
        try:
            motor = max(0, min(255, int(value)))
        except (TypeError, ValueError):
            self.get_logger().warn(f'Invalid value for {side} motor: {value}')
            return 0

        if side == 'left':
            self.dualsense.setLeftMotor(motor)
        else:
            self.dualsense.setRightMotor(motor)
        return motor

    def _on_left_rumble(self, msg: Float32) -> None:
        self.left_motor_power = self._apply_motor_power('left', self._normalize_motor_command(msg.data))

    def _on_right_rumble(self, msg: Float32) -> None:
        self.right_motor_power = self._apply_motor_power('right', self._normalize_motor_command(msg.data))

    def _normalize_motor_command(self, value: float) -> int:
        scaled = int(math.floor(max(0.0, min(1.0, float(value))) * 255.0))
        return scaled

    def _on_light_color(self, msg: ColorRGBA) -> None:
        rgb = (
            int(max(0, min(255, round(msg.r * 255 if msg.r <= 1.0 else msg.r)))),
            int(max(0, min(255, round(msg.g * 255 if msg.g <= 1.0 else msg.g)))),
            int(max(0, min(255, round(msg.b * 255 if msg.b <= 1.0 else msg.b)))),
        )
        self.indicator_on_color = rgb
        self._update_indicator_light(True, force=True)

    def _on_player_id_cmd(self, msg: UInt8) -> None:
        player_value = int(msg.data)
        lookup: Dict[int, PlayerID] = {
            0: PlayerID.ALL,
            1: PlayerID.PLAYER_1,
            2: PlayerID.PLAYER_2,
            3: PlayerID.PLAYER_3,
            4: PlayerID.PLAYER_4,
        }
        player = lookup.get(player_value)
        if player is None:
            self.get_logger().warn(f'Unsupported player ID: {player_value}')
            return
        self.dualsense.light.setPlayerID(player)

    def _on_light_brightness_cmd(self, msg: String) -> None:
        self._apply_light_brightness(msg.data)

    def _on_microphone_mute(self, msg: Bool) -> None:
        try:
            self.dualsense.audio.setMicrophoneState(bool(msg.data))
        except Exception as exc:  # pragma: no cover - hardware dependent
            self.get_logger().warn(f'Failed to configure microphone mute state: {exc}')

    def _on_microphone_led(self, msg: Bool) -> None:
        try:
            self.dualsense.audio.setMicrophoneLED(bool(msg.data))
        except Exception as exc:  # pragma: no cover - hardware dependent
            self.get_logger().warn(f'Failed to configure microphone LED state: {exc}')

    def _on_trigger_mode_left(self, msg: String) -> None:
        self.left_trigger_mode = msg.data
        self._apply_trigger_mode('left', self.left_trigger_mode)

    def _on_trigger_mode_right(self, msg: String) -> None:
        self.right_trigger_mode = msg.data
        self._apply_trigger_mode('right', self.right_trigger_mode)

    def _apply_trigger_mode(self, side: str, mode_name: str, silent: bool = False) -> None:
        mode = self._resolve_trigger_mode(mode_name)
        if mode is None:
            if not silent:
                self.get_logger().warn(f'Unknown trigger mode: {mode_name}')
            return
        trigger = self.dualsense.triggerL if side == 'left' else self.dualsense.triggerR
        try:
            trigger.setMode(mode)
        except Exception as exc:  # pragma: no cover - hardware dependent
            self.get_logger().warn(f'Failed to set {side} trigger mode: {exc}')

    def _resolve_trigger_mode(self, mode_name: str) -> Optional[TriggerModes]:
        key = mode_name.strip().upper()
        return self._trigger_modes_map.get(key)

    def _apply_light_brightness(self, value: str, silent: bool = False) -> None:
        key = value.strip().lower()
        mapping = {
            'low': Brightness.low,
            'medium': Brightness.medium,
            'high': Brightness.high,
        }
        brightness = mapping.get(key)
        if brightness is None:
            if not silent:
                self.get_logger().warn(f'Unknown light brightness: {value}')
            return
        try:
            self.dualsense.light.setBrightness(brightness)
        except Exception as exc:  # pragma: no cover - hardware dependent
            self.get_logger().warn(f'Failed to set light brightness: {exc}')

    def _apply_player_id(self, value: str, silent: bool = False) -> None:
        key = value.strip().upper()
        player = PlayerID.__members__.get(key)
        if player is None:
            if not silent:
                self.get_logger().warn(f'Unknown player ID: {value}')
            return
        try:
            self.dualsense.light.setPlayerID(player)
        except Exception as exc:  # pragma: no cover - hardware dependent
            self.get_logger().warn(f'Failed to set player ID: {exc}')

    def _convert_battery_status(self, ds_status: BatteryState) -> int:
        mapping = {
            BatteryState.POWER_SUPPLY_STATUS_CHARGING: BatteryStateMsg.POWER_SUPPLY_STATUS_CHARGING,
            BatteryState.POWER_SUPPLY_STATUS_DISCHARGING: BatteryStateMsg.POWER_SUPPLY_STATUS_DISCHARGING,
            BatteryState.POWER_SUPPLY_STATUS_FULL: BatteryStateMsg.POWER_SUPPLY_STATUS_FULL,
            BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING: BatteryStateMsg.POWER_SUPPLY_STATUS_NOT_CHARGING,
        }
        return mapping.get(ds_status, BatteryStateMsg.POWER_SUPPLY_STATUS_UNKNOWN)

    def shutdown(self) -> None:
        try:
            self.dualsense.close()
        except Exception as exc:  # pragma: no cover - hardware dependent
            self.get_logger().error(f'Failed to shut down DualSense: {exc}')


def main() -> None:
    rclpy.init()
    node = DualSenseDriver()
    node.get_logger().info('DualSense driver started.')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
