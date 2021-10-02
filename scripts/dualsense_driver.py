#!/usr/bin/env python3

import rospy
from pydualsense import pydualsense, TriggerModes
from sensor_msgs.msg import Joy

class DualSenseDriver:
    def __init__(self):
        rospy.init_node('dualsense_driver')
        self.dualsense = pydualsense()
        self.dualsense.init()
        self.joy_pub = rospy.Publisher('/joy', Joy, queue_size=10)

    def update(self):
        joy = Joy()
        joy.axes.clear()
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

        joy.buttons.clear()
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

    dualsense = DualSenseDriver()
    print("dualsense driver started")

    rate = rospy.Rate(60)
    while not rospy.is_shutdown():
        dualsense.update()
        rate.sleep()

    dualsense.shutdown()
    print("dualsense driver stopped")
