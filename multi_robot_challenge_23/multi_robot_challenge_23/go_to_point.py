import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from tf_transformations import euler_from_quaternion


class GoToPoint(Node):
    def __init__(self):
        super().__init__("go_to_point")
        
        self.declare_parameter("cmd_topic", "cmd_vel")
        self.declare_parameter("scan_topic", "scan")
        self.declare_parameter("odom_topic", "odom")
        self.declare_parameter("goal_topic", "goal")
        self.declare_parameter("goal_reached_topic", "goal_reached")

        cmd_topic = self.get_parameter("cmd_topic").value
        scan_topic = self.get_parameter("scan_topic").value
        odom_topic = self.get_parameter("odom_topic").value
        goal_topic = self.get_parameter("goal_topic").value
        goal_reached_topic = self.get_parameter("goal_reached_topic").value

        self.declare_parameter("wall_follow", True)     # aktiver wallfollow
        self.declare_parameter("front_distance", 0.5)   # terskel avstabd foran (m)
        self.declare_parameter("side_distance", 0.45)   # ønsket walldistance (m)
        self.declare_parameter("side", "right")         # 'right' eller 'left'
        self.declare_parameter("velocity_goal", 0.40)   # fart mot goal
        self.declare_parameter("velocity_wall", 0.20)   # fart langs vegg
        self.declare_parameter("max_ang_vel", 1.0)      # maks vinkelhastighet, rotering
        self.declare_parameter("kp", 1.2)               # P-vekt
        self.declare_parameter("kd", 0.15)              # D-vekt
        self.declare_parameter("goal_tolerance", 0.15)  # mål-toleranse (m)c

        # Tilstand
        self.mode = "GOTO"           # "GOTO" / "FOLLOW"
        self.last_scan = None
        self.prev_err = 0.0
        self.prev_t = self.get_clock().now().nanoseconds / 1e9

        self.target_x = None         # må settes av goal-sub
        self.target_y = None
        self.position = None
        self.yaw = None

        self.odom_sub = self.create_subscription(Odometry, odom_topic, self.clbk_odom, 10)
        self.scan_sub = self.create_subscription(LaserScan, scan_topic, self.clbk_scan, 10)
        self.goal_sub = self.create_subscription(PoseStamped, goal_topic, self.clbk_goal, 10)

        self.cmd_vel_pub = self.create_publisher(Twist, cmd_topic, 10)
        self.goal_pub = self.create_publisher(Bool, goal_reached_topic, 10)

        #logger
        self.get_logger().info(
            f"Topics: odom={odom_topic}, scan={scan_topic}, cmd={cmd_topic}, "
            f"goal_in={goal_topic}, goal_out={goal_reached_topic}"
        )

        self.timer = self.create_timer(0.1, self.timer_callback)

    # callbacks
    def clbk_scan(self, msg: LaserScan):
        self.last_scan = msg

    def clbk_odom(self, msg: Odometry):
        self.position = msg.pose.pose.position
        q = msg.pose.pose.orientation
        euler = euler_from_quaternion((q.x, q.y, q.z, q.w))
        self.yaw = euler[2]

    def clbk_goal(self, msg: PoseStamped):
        self.target_x = msg.pose.position.x
        self.target_y = msg.pose.position.y
        self.mode = "GOTO"  # 
        self.get_logger().info(f"Nytt mål: ({self.target_x:.2f}, {self.target_y:.2f})") # log

    # hjelpefunksjoner
    def sector_min(self, scan: LaserScan, deg_left: float, deg_right: float) -> float:
        """Min avstand i en vinkel-sektor (grader)."""
        if scan is None:
            return float("inf")

        def deg_to_index(deg):
            a = math.radians(deg)
            i = int((a - scan.angle_min) / scan.angle_increment)
            return max(0, min(len(scan.ranges) - 1, i))

        i0 = deg_to_index(deg_left)
        i1 = deg_to_index(deg_right)
        if i0 > i1:
            i0, i1 = i1, i0

        vals = [r for r in scan.ranges[i0:i1 + 1] if scan.range_min < r < scan.range_max]
        return min(vals) if vals else float("inf")

    def follow_wall_control(self, scan: LaserScan):
        """Enkel PD-kontroll for å holde ønsket avstand til vegg."""
        side = self.get_parameter("side").value
        d_ref = float(self.get_parameter("side_distance").value)
        kp = float(self.get_parameter("kp").value)
        kd = float(self.get_parameter("kd").value)
        max_w = float(self.get_parameter("max_ang_vel").value)

        if side == "right":
            d_side = self.sector_min(scan, -100, -60)
            sign = +1.0  # for liten avstand → sving venstre (positiv z)
        else:
            d_side = self.sector_min(scan, 60, 100)
            sign = -1.0

        err = (d_side - d_ref) * sign
        t = self.get_clock().now().nanoseconds / 1e9
        dt = max(1e-3, t - self.prev_t)
        derr = (err - self.prev_err) / dt

        w = kp * err + kd * derr
        w = max(-max_w, min(max_w, w))
        self.prev_err, self.prev_t = err, t

        v = float(self.get_parameter("velocity_wall").value)
        return v, w

    # main loop
    def timer_callback(self):
        if self.position is None or self.yaw is None:
            return

        # Ingen mål ennå → stå i ro
        if self.target_x is None or self.target_y is None:
            self.cmd_vel_pub.publish(Twist())
            return

        wall_on = bool(self.get_parameter("wall_follow").value)
        scan = self.last_scan
        front_dist_thr = float(self.get_parameter("front_distance").value)
        tol = float(self.get_parameter("goal_tolerance").value)

        # Beregn grunnleggende GOTO-størrelser
        dx = self.target_x - self.position.x
        dy = self.target_y - self.position.y
        distance = math.hypot(dx, dy)
        angle = math.atan2(dy, dx)
        angle_err = math.atan2(math.sin(angle - self.yaw), math.cos(angle - self.yaw))

        # Sjekk om goal er nådd
        if distance <= tol:
            self.cmd_vel_pub.publish(Twist())
            self.goal_pub.publish(Bool(data=True))
            # Slipp målet; koordinator sender nytt
            self.target_x = None
            self.target_y = None
            return

        # Hvis veggfølging er av eller ingen scan → ren GOTO
        if (not wall_on) or (scan is None):
            self.cmd_vel_pub.publish(self.goto_twist(distance, angle_err))
            self.mode = "GOTO"
            return

        # Veggfølging på → vurder tilstandsbytte
        d_front = self.sector_min(scan, -20, 20)

        if self.mode == "GOTO":
            if d_front < front_dist_thr:
                self.mode = "FOLLOW"

        elif self.mode == "FOLLOW":
            if d_front >= front_dist_thr * 1.4:
                self.mode = "GOTO"

        # Utfør valgt modus
        if self.mode == "GOTO":
            self.cmd_vel_pub.publish(self.goto_twist(distance, angle_err))
        else:
            v, w = self.follow_wall_control(scan)
            twist = Twist()
            twist.linear.x = v
            twist.angular.z = w
            self.cmd_vel_pub.publish(twist)

    def goto_twist(self, distance: float, angle_err: float) -> Twist:
       
        twist = Twist()
        v_goal = float(self.get_parameter("velocity_goal").value)

        # roter til kurs ~riktig, ellers kjør frem
        if abs(angle_err) < 0.2:
            if distance > 0.1:
                twist.linear.x = v_goal
                twist.angular.z = 0.0
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.2 if angle_err > 0.0 else -0.2

        return twist


def main(args=None):
    rclpy.init(args=args)
    node = GoToPoint()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
