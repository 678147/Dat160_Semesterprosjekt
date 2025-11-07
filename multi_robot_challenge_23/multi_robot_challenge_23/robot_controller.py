import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import math

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        

        self.declare_parameter('robot_name', 'robot1')
        robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
        ns = f'/{robot_name}'

        cmd_topic = f'{ns}/cmd_vel'
        scan_topic = f'{ns}/scan'
        odom_topic = f'{ns}/odom'
        status_topic = f'{ns}/status'

        self.cmd_vel_pub = self.create_publisher(Twist, cmd_topic, 10)
        self.status_pub = self.create_publisher(String, status_topic, 10)
        
        self.laser_sub = self.create_subscription(LaserScan, scan_topic, self.laser_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)
        
        self.current_pose = None
        self.min_distance = float('inf')
        self.safe_distance = 0.5
        self.get_logger().info(f'RobotController for {robot_name} initialized.')

        self.create_timer(0.1, self.publish_status)
        
    def laser_callback(self, msg):
        vals = [r for r in msg.ranges if math.isfinite(r)]
        self.min_distance = min(vals) if vals else float('inf')
    
    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        
    def publish_status(self):
        msg = String()
        d = self.min_distance if math.isfinite(self.min_distance) else -1.0
        msg.data = f"alive; min_dist={d:.2f}"
        self.status_pub.publish(msg)
        
        
def main(args=None):
    rclpy.init(args=args)
    robot_controller = RobotController()
    rclpy.spin(robot_controller)
    robot_controller.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()