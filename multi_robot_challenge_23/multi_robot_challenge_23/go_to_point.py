import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from tf_transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
import math

class goToPoint(Node):
    def __init__(self):
        super().__init__("bug2_navigation")

        self.target_x = 4.0
        self.target_y = 2.0

        self.position = None
        self.yaw = None

        self.odom_sub = self.create_subscription(Odometry, "/odom", self.clbk_odom, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.timer = self.create_timer(0.1, self.timer_callback) 

    
    def clbk_odom(self, msg):
        self.position =msg.pose.pose.position
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)
        euler = euler_from_quaternion(quaternion)
        self.yaw = euler[2]
    
    def timer_callback(self):
        if self.position is None or self.yaw is None:
            return

        vel_msg = Twist()

        vel_msg.linear.x = 0.0
        vel_msg.angular.z = 0.0
        dx = self.target_x - self.position.x
        dy = self.target_y - self.position.y
        distance = math.hypot(dx, dy)
        angle = math.atan2(dy, dx)

        angle_discrepancy = math.atan2(math.sin(angle-self.yaw), math.cos(angle-self.yaw))

        #on the right course
        if abs(angle_discrepancy) < 0.2:
            if distance > 0.1:
                vel_msg.linear.x = 0.4
            else:
                vel_msg.linear.x = 0.0
                vel_msg.angular.z = 0.0

        #find the right course
        else:
            vel_msg.linear.x = 0.0
            if angle_discrepancy > 0:
                vel_msg.angular.z = 0.2
            else:
                vel_msg.angular.z = -0.2
            
        self.cmd_vel_pub.publish(vel_msg)

def main(args=None):
    rclpy.init(args=args)

    controller = goToPoint()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


