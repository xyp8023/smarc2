#!/usr/bin/python3
import sys
import rclpy
from rclpy.node import Node

from smarc_msgs.msg import ThrusterRPM
from sam_msgs.msg import Topics as SamTopics

try:
    from .IThrustView import IThrustView
except:
    from IThrustView import IThrustView

class SAMThrustView(IThrustView):
    def __init__(self, node: Node) -> None:
        self._rpm1_pub = node.create_publisher(ThrusterRPM, SamTopics.THRUSTER1_CMD_TOPIC, 10)
        self._rpm2_pub = node.create_publisher(ThrusterRPM, SamTopics.THRUSTER2_CMD_TOPIC, 10)
        self._t1_msg = ThrusterRPM()
        self._t2_msg = ThrusterRPM()

    def set_rpm(self, rpm: int) -> None:
         self._t1_msg.rpm = rpm
         self._t2_msg.rpm = rpm

    def update(self) -> None:
        self._rpm1_pub.publish(self._t1_msg)
        self._rpm2_pub.publish(self._t2_msg)


def test_view():
    rclpy.init(args=sys.argv)
    node = rclpy.create_node("SAMThrustView_test")
    view = SAMThrustView(node)

    rpm = 500

    def loop():
        nonlocal rpm
        rpm *= -1
        view.set_rpm(rpm)
        view.update()

    loop_period = 1
    node.create_timer(loop_period, loop)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

  


if __name__ == "__main__":
    test_view()