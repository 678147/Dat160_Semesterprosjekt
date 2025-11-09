import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool


class WallFollower(Node):
    def __init__(self):
        super().__init__('wall_follower')
        self.get_logger().info('Startet WallFollower.')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.clbk_scan, qos_profile_sensor_data)

        self.srv = self.create_service(SetBool, 'wall_follow', self.handle_srv)

       
        self.following = False
        self.wall_distance = 0.5
        self.front_threshold = 0.6
        self.turn_speed = 0.45
        self.forward_speed = 0.29
        self.turning_counter = 0

    def handle_srv(self, req, res):
        self.following = req.data
        if not self.following:
            self.cmd_pub.publish(Twist()) 
        res.success = True
        res.message = 'WallFollower nå startet' if self.following else 'WallFollower er IKKE aktiv'
        return res

    @staticmethod
    def _idx_for_angle(msg: LaserScan, ang: float) -> int:
        
        return int(round((ang - msg.angle_min) / msg.angle_increment))

    def _window(self, msg: LaserScan, ang_center: float, width_rad: float):
        
        i_center = self._idx_for_angle(msg, ang_center)
        half = max(1, int(round(width_rad / msg.angle_increment / 2)))
        n = len(msg.ranges)
        inds = [(i_center + k) % n for k in range(-half, half + 1)]
        vals = []
        for i in inds:
            v = msg.ranges[i]
            if math.isfinite(v):
                vals.append(v)
        return vals

    def clbk_scan(self, msg: LaserScan):
        if not self.following:
            return

        
        front = self._window(msg, 0.0, math.radians(30))          
        right = self._window(msg, -math.pi / 2, math.radians(40)) 

        front_min = min(front) if front else float('inf')
        right_min = min(right) if right else float('inf')

        cmd = Twist()

        if front_min < self.front_threshold:
            
            cmd.linear.x = 0.0
            cmd.angular.z = self.turn_speed
            self.turning_counter = 0
        elif right_min < self.wall_distance:
            
            cmd.linear.x = self.forward_speed
            cmd.angular.z = 0.1
            self.turning_counter = 0
        elif right_min > self.wall_distance:
          
            if self.turning_counter < 5:
                cmd.linear.x = 0.2
                cmd.angular.z = -self.turn_speed
                self.turning_counter += 1
            else:
                self.turning_counter = 0
        else:
        
            cmd.linear.x = self.forward_speed
            cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = WallFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
