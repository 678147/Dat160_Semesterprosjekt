import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped


class Bug2Controller(Node):
    def __init__(self):
        super().__init__('bug2_controller')

        self.go_active_pub = self.create_publisher(Bool, '/go_to_point/active', 10)

        self.wall_follower_client = self.create_client(SetBool, 'wall_follow')
        
        self.get_logger().info('Venter på WF')

        self.laser_sub = self.create_subscription(LaserScan, '/scan', self.clbk_laser, qos_profile_sensor_data)
        self.goal_sub = self.create_subscription(PoseStamped, '/move_base_simple/goal', self.clbk_goal, 10)

    
        self.reached_sub = self.create_subscription(Bool, '/go_to_point/reached', self.clbk_reached, 10)
        
        # States
        self.state = 'idle'          # 'idle' / 'go_to_point' / 'wall_following'
        self.has_goal = False
        self.target_pose = None
        self.reached = False

        # Parms
        self.obstacle_dist = 0.5   

        self.get_logger().info('Bug2Controller ready: Publish til /move_base_simple/goal.')

    def clbk_goal(self, msg: PoseStamped):
        self.target_pose = msg
        self.has_goal = True
        self.reached = False 
        self.get_logger().info(
            f"Nytt PS-target: {msg.header.frame_id} "
            f"x:{msg.pose.position.x:.2f}, y:{msg.pose.position.y:.2f}"
        )
        self.switch_to_go_to_point()

    def clbk_reached(self, msg: Bool):
        self.reached = msg.data
        if self.reached:
            
            self.get_logger().info('Mål nådd - switcher til idle')
            self.go_active_pub.publish(Bool(data=False))  
            self._call_wall_follower(False)               
            self.state = 'idle'
            self.has_goal = False

    def clbk_laser(self, msg: LaserScan):
        if self.state == 'idle' or not self.has_goal or self.reached:
            return

        ranges = [r for r in msg.ranges if math.isfinite(r)]
        if not ranges:
            return
        min_distance = min(ranges)

        if self.state == 'go_to_point' and min_distance < self.obstacle_dist:
            self.get_logger().info('Hindring møtt, switcher til WallFollower.')
            self.switch_to_wall_follower()
        elif self.state == 'wall_following' and min_distance >= self.obstacle_dist + 0.1:
            self.get_logger().info('Hindring borte, switcher til GTP.')
            self.switch_to_go_to_point()

    def _call_wall_follower(self, enable: bool):
        req = SetBool.Request()
        req.data = enable
        self.wall_follower_client.call_async(req)

    def switch_to_go_to_point(self):
        if not self.has_goal:
            self.get_logger().warn('Ingen mål')
            return
        self.state = 'go_to_point'
        self._call_wall_follower(False)
        self.go_active_pub.publish(Bool(data=True))
        self.get_logger().info('Switchet til GTP.')

    def switch_to_wall_follower(self):
        self.state = 'wall_following'
        self.go_active_pub.publish(Bool(data=False))
        self._call_wall_follower(True)
        self.get_logger().info('Switchet til WallFollower.')


def main(args=None):
    rclpy.init(args=args)
    node = Bug2Controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
