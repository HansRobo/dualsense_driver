#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from pydualsense import pydualsense, TriggerModes
from sensor_msgs.msg import Joy

class DualSenseDriver(Node):
    def __init__(self):
        super().__init__('dualsense_driver')
        self.dualsense = pydualsense()
        self.dualsense.init()
        self.joy_pub = self.create_publisher(Joy, '/joy', 10)
        self.timer = self.create_timer(1.0/60.0, self.update)

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
