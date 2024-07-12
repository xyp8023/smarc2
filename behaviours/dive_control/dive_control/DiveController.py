#!/usr/bin/python3

import rclpy, sys, time
from rclpy.node import Node

import numpy as np
import math

import tf2_geometry_msgs.tf2_geometry_msgs
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TransformStamped

from control_msgs.msg import Topics as ControlTopics

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from tf_transformations import euler_from_quaternion

from .IDiveView import IDiveView

class DiveController():
    """
    Dive Controller to listen to a waypoint and provide the corresponding setpoints to the
    DivingModel within the MVC framework
    """
    def __init__(self,
                 node: Node,
                 view: IDiveView):

        self._node = node
        self._view = view

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self._node)

        # TODO: Can we get this from a launch file or so?
        self._robot_base_link = 'sam0/base_link_gt'

        # TODO: Update with waypoint following
        self._depth_setpoint = None
        self._pitch_setpoint = None
        self._requested_rpm = None
        self._waypoint_global = None
        self._waypoint_body = None
        self._received_waypoint = False

        self._tf_base_link = None

        self._states = Odometry()

        # TODO: waypoint topic
        self.depth_sub = node.create_subscription(msg_type=Float64, topic=ControlTopics.DEPTH_SETPOINT, callback=self._depth_cb, qos_profile=10)
        self.pitch_sub = node.create_subscription(msg_type=Float64, topic=ControlTopics.PITCH_SETPOINT, callback=self._pitch_cb, qos_profile=10)
        self.state_sub = node.create_subscription(msg_type=Odometry, topic=ControlTopics.STATES, callback=self._states_cb, qos_profile=10)
        self.waypoint_sub = node.create_subscription(msg_type=Odometry, topic='/ctrl/waypoint', callback=self._wp_cb, qos_profile=10)

        self._loginfo("Dive Controller Node started")


    def _loginfo(self, s):
        self._node.get_logger().info(s)


    def _depth_cb(self, depth):
        self._depth_setpoint = depth.data


    def _pitch_cb(self, pitch):
        self._pitch_setpoint = pitch.data


    def _states_cb(self, msg):
        self._states = msg


    def _wp_cb(self, wp):
        # TODO: This needs to be first transformed into the body frame!
        # but not in the callback, but in the update function so that it's
        # always up to date with the latest position of SAM!
        self._waypoint_global = PoseStamped()
        self._waypoint_global.header.stamp = wp.header.stamp
        self._waypoint_global.header.frame_id = wp.header.frame_id
        self._waypoint_global.pose.position.x = wp.pose.pose.position.x
        self._waypoint_global.pose.position.y = wp.pose.pose.position.y
        self._waypoint_global.pose.position.z = wp.pose.pose.position.z
        self._waypoint_global.pose.orientation.x = wp.pose.pose.orientation.x
        self._waypoint_global.pose.orientation.y = wp.pose.pose.orientation.y
        self._waypoint_global.pose.orientation.z = wp.pose.pose.orientation.z
        self._waypoint_global.pose.orientation.w = wp.pose.pose.orientation.w

        #TODO: Get the proper RPM from the waypoint
        self._requested_rpm = 500

        self._received_waypoint = True


    def _update_tf(self):
        if self._waypoint_global is None:
            return

        try:
            self._tf_base_link = self._tf_buffer.lookup_transform(self._robot_base_link,
                                                                  self._waypoint_global.header.frame_id,
                                                                  rclpy.time.Time(seconds=0))
        except Excepetion as ex:
            self._loginfo(
                f"Could not transform {self._robot_base_link} to {self._waypoint_global.header.frame_id}: {ex}")
            return


    def _transform_wp(self):
        if self._waypoint_global is None:
            return

        if self._tf_base_link is None:
            return

        self._waypoint_body = tf2_geometry_msgs.do_transform_pose(self._waypoint_global.pose, self._tf_base_link)


    def get_depth_setpoint(self):
        if self._waypoint_body is not None:
            self._depth_setpoint = self._waypoint_global.pose.position.z

        return self._depth_setpoint


    def get_pitch_setpoint(self):
        if self._waypoint_body is not None:
            rpy = euler_from_quaternion([
                self._waypoint_global.pose.orientation.x,
                self._waypoint_global.pose.orientation.y,
                self._waypoint_global.pose.orientation.z,
                self._waypoint_global.pose.orientation.w])

            self._pitch_setpoint = rpy[1]

        return self._pitch_setpoint

    def get_heading_setpoint(self):
        return 0.0

    def get_rpm_setpoint(self):
        return self._requested_rpm

    def get_states(self):
        # TODO: Might be better to split this by what 
        # state you're interested in, then you can get them
        # directly.
        return self._states


    def get_depth(self):
        return self._states.pose.pose.position.z


    def get_pitch(self):

        rpy = euler_from_quaternion([
            self._states.pose.pose.orientation.x,
            self._states.pose.pose.orientation.y,
            self._states.pose.pose.orientation.z,
            self._states.pose.pose.orientation.w])

        return rpy[1]


    def get_heading(self):

        if self._waypoint_body is None:
            return None

        heading = math.atan2(self._waypoint_body.position.y, self._waypoint_body.position.x)

        return heading

    def get_distance(self):

        if self._waypoint_body is None:
            return None

        distance = math.sqrt(self._waypoint_body.position.x**2 + self._waypoint_body.position.y**2)

        return distance

    def get_dive_pitch(self):
        if self._waypoint_body is None:
            return None

        # With the ata2, we automatically get the desired diving pitch angle that corresponds to 
        # a ENU system, i.e. positive pitch for diving down, negative pitch for diving up
        current_depth = self.get_depth()
        depth_error = np.abs(self._waypoint_global.pose.position.z) - np.abs(current_depth)
        distance = self.get_distance()
        dive_pitch = math.atan2(depth_error, distance)

        self._loginfo(f"depth error: {depth_error:.3f}, pitch: {dive_pitch}, distance: {distance:.3f}")
        #self._loginfo(f"depth: {current_depth}, z_global: {self._waypoint_global.pose.position.z}, z_body: {self._waypoint_body.position.z}")


        return dive_pitch

    def has_waypoint(self):
        return self._received_waypoint

    def get_waypoint(self):
        return self._waypoint_global


    def update(self):
        """
        All the things when updating
        """
        self._update_tf()
        self._transform_wp()



def main():
#    # when creating the _object_ rather than the _class_, we use the concrete classes
#    from .SAMDiveView import SAMThrustView
#
    # create a node and our objects in the usual manner.
    rclpy.init(args=sys.argv)
    node = rclpy.create_node("DiveNode")
    node._logger("not implemented")
#    view = SAMThrustView(node)
#    controller = GoToWaypointActionServerController(node, view)
#
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)


if __name__ == "__main__":
    main()