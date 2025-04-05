#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from pydualsense import pydualsense, TriggerModes
import pydualsense.enums as enums
from sensor_msgs.msg import Joy, BatteryState
from rcl_interfaces.msg import SetParametersResult

class DualSenseDriver(Node):
    def __init__(self):
        super().__init__('dualsense_driver')
        self.dualsense = pydualsense()
        self.dualsense.init()
        self.joy_pub = self.create_publisher(Joy, '/joy', 10)
        self.battery_pub = self.create_publisher(BatteryState, '/battery', 10)
        self.timer = self.create_timer(1.0/60.0, self.update)

        self.right_motor_power = self.declare_parameter('right_motor', 0).value
        self.left_motor_power = self.declare_parameter('left_motor', 0).value

        self.add_on_set_parameters_callback(self.on_set_parameter_callback)

    def on_set_parameter_callback(self, parameter_list):
        for param in parameter_list:
            if param.name == 'right_motor':
                self.dualsense.setRightMotor(param.value)
            elif param.name == 'left_motor':
                self.dualsense.setLeftMotor(param.value)
        return SetParametersResult(successful=True)

    def update(self):
        joy = Joy()
        joy.axes = []
        joy.axes.append(-(self.dualsense.state.LX)/128.0)
        joy.axes.append(-(self.dualsense.state.LY)/128.0)
        joy.axes.append(-(self.dualsense.state.RX)/128.0)
        joy.axes.append(-(self.dualsense.state.RY)/128.0)
        joy.axes.append((self.dualsense.state.L2)/256.0)
        joy.axes.append((self.dualsense.state.R2)/256.0)
        joy.axes.append(0.0)
        joy.axes.append(0.0)
        joy.axes.append(0.0)

        if self.dualsense.state.DpadRight:
            joy.axes.append(1)
        elif self.dualsense.state.DpadLeft:
            joy.axes.append(-1)
        else:
            joy.axes.append(0)

        if self.dualsense.state.DpadUp:
            joy.axes.append(1)
        elif self.dualsense.state.DpadDown:
            joy.axes.append(-1)
        else:
            joy.axes.append(0)

        joy.buttons = []
        joy.buttons.append(self.dualsense.state.square)
        joy.buttons.append(self.dualsense.state.cross)
        joy.buttons.append(self.dualsense.state.circle)
        joy.buttons.append(self.dualsense.state.triangle)
        joy.buttons.append(self.dualsense.state.L1)
        joy.buttons.append(self.dualsense.state.R1)
        joy.buttons.append(self.dualsense.state.L2)
        joy.buttons.append(self.dualsense.state.R2)
        joy.buttons.append(self.dualsense.state.share)
        joy.buttons.append(self.dualsense.state.options)
        joy.buttons.append(self.dualsense.state.R3)
        joy.buttons.append(self.dualsense.state.L3)
        joy.buttons.append(self.dualsense.state.ps)
        joy.buttons.append(self.dualsense.state.touchBtn)
        self.joy_pub.publish(joy)
        self.battery_pub.publish(self.get_battery_state_msg())

    def get_battery_state_msg(self):
        battery_state_msg = BatteryState()
        battery_state = self.dualsense.battery.State

        if battery_state == enums.BatteryState.POWER_SUPPLY_STATUS_CHARGING:
            battery_state_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
        elif battery_state == enums.BatteryState.POWER_SUPPLY_STATUS_DISCHARGING:
            battery_state_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        elif battery_state == enums.BatteryState.POWER_SUPPLY_STATUS_FULL:
            battery_state_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_FULL
        elif battery_state == enums.BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING:
            battery_state_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING
        else:
            battery_state_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN

        battery_state_msg.present = True  # controller connected
        battery_state_msg.percentage = self.dualsense.battery.Level / 100.0 # pydualsense report range is 0-100
        battery_state_msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        battery_state_msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD

        battery_state_msg.voltage = float('nan')
        if battery_state_msg.percentage <= 0.0:
            battery_state_msg.voltage = 3.0
        elif battery_state_msg.percentage < 0.1:
            battery_state_msg.voltage = 3.5
        elif battery_state_msg.percentage < 0.2:
            battery_state_msg.voltage = 3.7
        elif battery_state_msg.percentage < 0.3:
            battery_state_msg.voltage = 3.8
        elif battery_state_msg.percentage < 0.4:
            battery_state_msg.voltage = 3.9
        elif battery_state_msg.percentage < 0.5:
            battery_state_msg.voltage = 3.95
        elif battery_state_msg.percentage < 0.6:
            battery_state_msg.voltage = 4.0
        elif battery_state_msg.percentage < 0.7:
            battery_state_msg.voltage = 4.05
        elif battery_state_msg.percentage < 0.8:
            battery_state_msg.voltage = 4.1
        elif battery_state_msg.percentage < 0.9:
            battery_state_msg.voltage = 4.15
        else:
            battery_state_msg.voltage = 4.2
        battery_state_msg.temperature = float('nan')
        battery_state_msg.current = float('nan')
        battery_state_msg.charge = float('nan')
        battery_state_msg.capacity = float('nan')
        battery_state_msg.design_capacity = float('nan')
        battery_state_msg.location = ""
        battery_state_msg.serial_number = ""

        battery_state_msg.header.stamp = self.get_clock().now().to_msg()
        battery_state_msg.header.frame_id = "dualsense_controller"
        return battery_state_msg
    def shutdown(self):
        self.dualsense.close()

if __name__ == '__main__':

    rclpy.init(args=None)
    dualsense = DualSenseDriver()
    print("dualsense driver started")

    try:
        rclpy.spin(dualsense)
    except KeyboardInterrupt:
        pass
    finally:
        dualsense.shutdown()
        dualsense.destroy_node()
        rclpy.shutdown()
        print("dualsense driver stopped")
