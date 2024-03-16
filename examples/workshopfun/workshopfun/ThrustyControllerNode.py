#!/usr/bin/python3

import sys
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, IntegerRange
from example_interfaces.srv import SetBool

from .IThrustView import IThrustView
from .advanced_science.IRPMModel import IRPMModel

class ThrustyControllerNode:
    def __init__(self,
                 node: Node,
                 view: IThrustView,
                 model: IRPMModel):

        self._node = node
        self._view = view
        self._model = model

        min, max, step, default = 0, 1000, 100, 500

        rpm_range = IntegerRange(from_value=min, to_value=max, step=step)
        rpm_limit_descriptor = ParameterDescriptor(
            name = "rpm_limit",
            description = "The upper and lower limits given to the Advanced Science model",
            integer_range = [rpm_range])
        self._rpm_limit_param = node.declare_parameter(
            name = 'rpm_limit',
            value = default,
            descriptor = rpm_limit_descriptor)
        
        self._model.set_range(self._rpm_limit_param.value)

        self._log_rpm_info_service = self._node.create_service(SetBool, "log_rpm_info", self._log_rpm_info_service_cb)
        self._log_rpm_info = True

    def _log_rpm_info_service_cb(self, request, response):
        self._log_rpm_info = request.data
        response.success = True
        response.message = f"Toggled logging the RPM changes to {request.data}!"
        return response

    def update(self):
        r = self._node.get_parameter('rpm_limit').get_parameter_value().integer_value
        self._model.set_range(r)
        if(self._log_rpm_info):
            self._node.get_logger().info(f"RPM Limit: {r}")
        rpm = self._model.computeRPM()

        if(self._log_rpm_info):
            self._node.get_logger().info(f"Setting RPM to {rpm}")
        self._view.set_rpm(rpm)
        self._view.update()

    def run(self):
        loop_period = 1
        self._node.create_timer(loop_period, self.update)
        try:
            rclpy.spin(self._node)
        except KeyboardInterrupt:
            pass

def main():
    from .advanced_science.SuperAdvancedScienceModel import SuperAdvancedScienceModel
    from .SAMThrustView import SAMThrustView

    rclpy.init(args=sys.argv)
    node = rclpy.create_node("AdvancedScienceNode")

    view = SAMThrustView(node)
    model = SuperAdvancedScienceModel(range=500)

    controller = ThrustyControllerNode(node, view, model)

    controller.run()



if __name__ == "__main__":
    main()