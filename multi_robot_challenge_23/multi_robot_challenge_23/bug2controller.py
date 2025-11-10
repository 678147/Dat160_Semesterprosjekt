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

        self.declare_parameter('robot_name', '')
        robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
        ns = f'/{robot_name}' if robot_name else ''

        #Publisher til gotopoint noden at den skal være active (true) eller inactive (false)
        self.go_active_pub = self.create_publisher(Bool, f'{ns}/go_to_point/active', 10)

        #Service for å slå WallFollower av og på (setbool=True/false)
        self.wall_follower_client = self.create_client(SetBool, f'{ns}/wall_follow')
        
        self.get_logger().info('Venter på WF')

        #Lytter på laserdata fra LIDAR, sensor qos fordi det kommer flere raske meldinger
        self.laser_sub = self.create_subscription(LaserScan, f'{ns}/scan', self.clbk_laser, qos_profile_sensor_data)
        
        # Målet (når vi klikker 2D Nav Goal i RViz eller publiserer PoseStamped manuelt)
        self.goal_sub = self.create_subscription(PoseStamped, f'{ns}/move_base_simple/goal', self.clbk_goal, 10)
    
        #lytter på gotopoint/reached etter beskjed om målet er nådd, True når det er nådd
        self.reached_sub = self.create_subscription(Bool, f'{ns}/go_to_point/reached', self.clbk_reached, 10)
        
        # States
        self.state = 'idle'          # 'idle' / 'go_to_point' / 'wall_following'
        self.has_goal = False       #har vi et mål?
        self.target_pose = None    #selve målet fra PoseStamped
        self.reached = False       #gotopoint meldt ifra om vi er fremme?

        # Parms
        self.obstacle_dist = 0.5   #hvor nært vi tillater at turtleboten skal kunne kjøre mot hindring før vi bruker wallfollower

        self.get_logger().info('Bug2Controller ready: Publish til /move_base_simple/goal.')

    #callbackfunksjon for å registrere nytt mål (rviz/kordinator), lagrer målet, nullstiller "reached" og ber gotopoint starte.
    def clbk_goal(self, msg: PoseStamped):
        self.target_pose = msg
        self.has_goal = True
        self.reached = False 
        self.get_logger().info(
            f"Nytt PoseStamped-target: {msg.header.frame_id} "
            f"x:{msg.pose.position.x:.2f}, y:{msg.pose.position.y:.2f}"
        )
        self.switch_to_go_to_point()

    #callback når gotpoint sier at et mål er nådd, da stopper vi og går idle
    def clbk_reached(self, msg: Bool):
        self.reached = msg.data
        if self.reached:
            
            self.get_logger().info('Mål nådd - switcher til idle')
            #deaktiver gotopoint
            self.go_active_pub.publish(Bool(data=False))
            #slå av wallfollower
            self._call_wall_follower(False)         
            #sett state til idle siden vi ikke har et mål      
            self.state = 'idle'
            self.has_goal = False

    #callbackfunksjon som kalles kontinuerlig med avstandsmålinger. Bestemmer om vi skal bytte mellom gotopoint og wallfollower
    def clbk_laser(self, msg: LaserScan):
        
        #ikke gjør noe hvis vi er idle, ikke har goal eller allerede er der
        if self.state == 'idle' or not self.has_goal or self.reached:
            return

        ranges = [r for r in msg.ranges if math.isfinite(r)]
        if not ranges:
            return
        min_distance = min(ranges)

        #Hvis vi kjører mot målet og noe er for nært foran bytter vi til wallfollower
        if self.state == 'go_to_point' and min_distance < self.obstacle_dist:
            self.get_logger().info('Hindring møtt, switcher til WallFollower.')
            self.switch_to_wall_follower()
        
        #Hvis vi følger vegg og det ikke lenger er en hindring, går vi tilbake til gotopoint
        elif self.state == 'wall_following' and min_distance >= self.obstacle_dist + 0.1:
            self.get_logger().info('Hindring borte, switcher til GTP.')
            self.switch_to_go_to_point()

    #hjelpefunksjon for å slå wallfollower av og på via servicecall
    def _call_wall_follower(self, enable: bool):
        req = SetBool.Request()
        req.data = enable
        self.wall_follower_client.call_async(req)

    #funksjon som aktiverer gotopoint og deaktiverer wallfollower
    def switch_to_go_to_point(self):
        
        if not self.has_goal:
            self.get_logger().warn('Ingen mål')
            return
        self.state = 'go_to_point'
        self._call_wall_follower(False)
        self.go_active_pub.publish(Bool(data=True))
        self.get_logger().info('Switchet til GTP.')

    #funksjon som aktiverer wallfollower og deaktiverer gotopoint
    def switch_to_wall_follower(self):
        self.state = 'wall_following'
        self.go_active_pub.publish(Bool(data=False))
        self._call_wall_follower(True)
        self.get_logger().info('Switchet til WallFollower.')

#start av bug2controller node
def main(args=None):
    rclpy.init(args=args)
    node = Bug2Controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
