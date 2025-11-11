import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import math

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')

        self.declare_parameter('robot_name', 'tb3_0')
        robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
        ns = f'/{robot_name}'

        cmd_topic = f'{ns}/cmd_vel'
        scan_topic = f'{ns}/scan'
        odom_topic = f'{ns}/odom'
        status_topic = f'{ns}/status'
        goal_topic = f'{ns}/move_base_simple/goal'

        self.cmd_vel_pub = self.create_publisher(Twist, cmd_topic, 10)
        self.status_pub = self.create_publisher(String, status_topic, 10)

        self.laser_sub = self.create_subscription(LaserScan, scan_topic, self.laser_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, goal_topic, self.goal_callback, 10)
        self.reached_sub = self.create_subscription(Bool, f'{ns}/go_to_point/reached', self.reached_callback, 10)

        self.current_pose = None
        self.min_distance = float('inf')
        self.safe_distance = 0.4

        # Goal navigation state
        self.active_goal = None  # PoseStamped
        self.state = 'idle'  # idle / moving / done

        # Navigation params
        self.heading_eps = 0.15
        self.dist_eps = 0.25
        self.angular_speed = 0.8
        self.linear_speed = 0.3

        self.get_logger().info(f'RobotController for {robot_name} initialized. Listening on {goal_topic}')

        self.create_timer(0.1, self._update)

        # publisher to activate go_to_point
        self.gtp_active_pub = self.create_publisher(Bool, f'{ns}/go_to_point/active', 10)
        # service client for wallfollower
        from std_srvs.srv import SetBool
        self.wall_client = self.create_client(SetBool, f'{ns}/wall_follow')

    def laser_callback(self, msg: LaserScan):
        vals = [r for r in msg.ranges if math.isfinite(r)]
        self.min_distance = min(vals) if vals else float('inf')

    def odom_callback(self, msg: Odometry):
        self.current_pose = msg.pose.pose

    def goal_callback(self, msg: PoseStamped):
        # accept a new goal (in map frame) published by coordinator
        self.active_goal = msg
        self.state = 'moving'
        # activate namespaced go_to_point
        self.gtp_active_pub.publish(Bool(data=True))
        self.get_logger().info(f'Received goal x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f} - activating go_to_point')

    def reached_callback(self, msg: Bool):
        if msg.data:
            self.get_logger().info('go_to_point reports goal reached')
            self.state = 'idle'
            # ensure wall follower is off
            try:
                from std_srvs.srv import SetBool
                if self.wall_client.wait_for_service(timeout_sec=0.5):
                    req = SetBool.Request()
                    req.data = False
                    self.wall_client.call_async(req)
            except Exception:
                pass

    @staticmethod
    def _wrap(a):
        while a > math.pi:
            a -= 2 * math.pi
        while a < -math.pi:
            a += 2 * math.pi
        return a

    def _stop(self):
        self.cmd_vel_pub.publish(Twist())

    def _publish_status(self):
        s = String()
        s.data = self.state
        self.status_pub.publish(s)

    def _update(self):
        # publish status periodically
        self._publish_status()

        # nothing to do
        if self.state != 'moving' or self.active_goal is None or self.current_pose is None:
            return

        # compute error
        dx = self.active_goal.pose.position.x - self.current_pose.position.x
        dy = self.active_goal.pose.position.y - self.current_pose.position.y
        dist = math.hypot(dx, dy)

        # compute yaw from quaternion
        q = self.current_pose.orientation
        # naive conversion
        yaw = 0.0
        try:
            from tf_transformations import euler_from_quaternion
            yaw = euler_from_quaternion((q.x, q.y, q.z, q.w))[2]
        except Exception:
            pass

        target_angle = math.atan2(dy, dx)
        heading_error = self._wrap(target_angle - yaw)

        cmd = Twist()

        # obstacle handling: if obstacle detected while moving, switch to wallfollower
        if self.state == 'moving' and self.min_distance < self.safe_distance:
            self.get_logger().info('Obstacle detected - switching to wallfollower')
            # deactivate go_to_point
            self.gtp_active_pub.publish(Bool(data=False))
            # call wall follower service to enable
            try:
                from std_srvs.srv import SetBool
                if self.wall_client.wait_for_service(timeout_sec=1.0):
                    req = SetBool.Request()
                    req.data = True
                    self.wall_client.call_async(req)
                    self.state = 'wall_following'
            except Exception as e:
                self.get_logger().warn(f'Failed to call wall_follow service: {e}')
            # stop local motion commands - wallfollower will publish cmd_vel
            self._stop()
            return

        # if we are wall following and obstacle is gone, switch back to go_to_point
        if self.state == 'wall_following' and self.min_distance >= (self.safe_distance + 0.15):
            self.get_logger().info('Obstacle cleared - switching back to go_to_point')
            # disable wall follower
            try:
                from std_srvs.srv import SetBool
                if self.wall_client.wait_for_service(timeout_sec=1.0):
                    req = SetBool.Request()
                    req.data = False
                    self.wall_client.call_async(req)
            except Exception as e:
                self.get_logger().warn(f'Failed to call wall_follow service: {e}')
            # reactivate go_to_point
            self.gtp_active_pub.publish(Bool(data=True))
            self.state = 'moving'
            return

        if abs(heading_error) > self.heading_eps:
            cmd.angular.z = max(-self.angular_speed, min(self.angular_speed, 1.5 * heading_error))
            cmd.linear.x = 0.0
        elif dist > self.dist_eps:
            cmd.linear.x = self.linear_speed
            cmd.angular.z = 0.0
        else:
            # reached
            self._stop()
            self.state = 'done'
            self.get_logger().info('Goal reached')
            self.active_goal = None

        self.cmd_vel_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    robot_controller = RobotController()
    rclpy.spin(robot_controller)
    robot_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()