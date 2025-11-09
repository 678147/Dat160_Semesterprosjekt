import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion


class GoToPoint(Node):
    def __init__(self):
        super().__init__('go_to_point')
        self.get_logger().info("Startet Go To Point-node.")

    
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.clbk_odom, qos_profile_sensor_data)
        self.goal_sub = self.create_subscription(PoseStamped, '/move_base_simple/goal', self.clbk_goal, 10)
        self.active_sub = self.create_subscription(Bool, '/go_to_point/active', self.clbk_active, 10)
        self.reached_pub = self.create_publisher(Bool, '/go_to_point/reached', 10)

        # States
        self.active = False
        self.target = None      
        self.position = None
        self.yaw = None
        self.reached_goal = True

        # Params
        self.heading_eps = 0.12 
        self.dist_eps = 0.12   
        self.angular_speed = 0.8
        self.linear_speed = 0.35

    def clbk_active(self, msg: Bool):
        self.active = msg.data
        if not self.active:
            self._stop()
            self.get_logger().info("GTP-node deaktivert.")
        else:
            self.get_logger().info("GTP-node aktivert.")

    def clbk_goal(self, msg: PoseStamped):
        self.target = msg.pose.position
        self.reached_goal = False
      
        self.reached_pub.publish(Bool(data=False))
        self.get_logger().info(f"POS: x={self.target.x:.2f}, y={self.target.y:.2f}")

    def clbk_odom(self, msg: Odometry):
        self.position = msg.pose.pose.position
        q = msg.pose.pose.orientation
        eul = euler_from_quaternion((q.x, q.y, q.z, q.w))
        self.yaw = eul[2]
        self.navigate()

    @staticmethod
    def _wrap(a):
        while a > math.pi:
            a -= 2 * math.pi
        while a < -math.pi:
            a += 2 * math.pi
        return a

    def _stop(self):
        self.cmd_pub.publish(Twist())

    def navigate(self):
        if not self.active or self.position is None or self.yaw is None or self.target is None or self.reached_goal:
            return

        dx = self.target.x - self.position.x
        dy = self.target.y - self.position.y
        target_angle = math.atan2(dy, dx)
        heading_error = self._wrap(target_angle - self.yaw)
        distance = math.hypot(dx, dy)

        cmd = Twist()
        if abs(heading_error) > self.heading_eps:
         
            cmd.angular.z = max(-self.angular_speed, min(self.angular_speed, 1.5 * heading_error))
        elif distance > self.dist_eps:
            cmd.linear.x = self.linear_speed
        else:
            self.reached_goal = True
            self._stop()
        
            self.reached_pub.publish(Bool(data=True))
            self.get_logger().info("Mål nådd.")
            return 

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = GoToPoint()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
