#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import TrajectorySetpoint

class SetpointAssignerNode(Node):
    def __init__(self):
        super().__init__('setpoint_assigner')
        self.publisher = self.create_publisher(TrajectorySetpoint, '/px4_1/fmu/in/trajectory_setpoint', 10)
        # self.timer = self.create_timer(1.0, self.publish_setpoint)
        

        msg = TrajectorySetpoint()
        # Need to look into time syncronization in the future: Rahul

        # msg.timestamp = self.get_clock().now().to_msg()

        # msg = Pose()
        while rclpy.ok():
            msg.position=[1.0, 1.0, -1.0]
            msg.velocity=[0.2,0.2,0.2]

            self.publisher.publish(msg)
            self.get_logger().info('Publishing setpoint')



def main(args=None):
    rclpy.init(args=args)
    setpoint_assigner_node = SetpointAssignerNode()
    rclpy.spin(setpoint_assigner_node)
    setpoint_assigner_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

