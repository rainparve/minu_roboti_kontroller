#!/usr/bin/env pyton3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from turtlesim.msg import Pose

class TurtleMappingNode(Node):

    def __init__ (self):
        super().__init__("mapping")
        self.get_logger().info("our mapping is started")
        
        self._pose_publisher = self.create_publisher(
            Twist, "/cmd_vel", 10)
        
        self._scan_listener = self.create_subscription(
            LaserScan, "/scan", self.robot_controller,10)
        
    def robot_controller(self, scan : LaserScan):
        cmd = Twist()

        a = 2

        self._front = min(scan.ranges[: a+1] + scan.ranges[-a:])
        self._left = min(scan.ranges[89-a : 89+a+1])
        self._back = min(scan.ranges[179-a : 179+a+1])
        self._right = min(scan.ranges[269-a : 269+a+1])


        if self._front < 1.0 or self._left < 0.3 or self._right < 0.3 :

            if self._left < self._right:

                cmd .linear.x = 0.05
                cmd .angular.z = -0.8
            else: 
                cmd.linear.x = 0.05
                cmd .angular.z = 0.8 


        else: 
            cmd.linear.x = 0.3
            cmd .angular.z = 0.0

        self._pose_publisher.publish(cmd)

    


def main(args=None):
    rclpy.init(args=args)
    node = TurtleMappingNode()
    rclpy.spin(node)
    rclpy.shutdown()