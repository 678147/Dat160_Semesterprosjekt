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

        #publisher til hjulkommando-topic, fart fremover og rotasjon til turtlebot
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        #Henter posisjon og retning fra tb hjul, odom.
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.clbk_odom, qos_profile_sensor_data)
        
        #Får målposisjon fra rviz (når det klikkes på nav goal pose eller sender posestamps i odom frame)
        self.goal_sub = self.create_subscription(PoseStamped, '/move_base_simple/goal', self.clbk_goal, 10)
        
        #subscriber på /go_to_point/active fra controller som bestemmer om tb skal være aktiv eller ikke
        self.active_sub = self.create_subscription(Bool, '/go_to_point/active', self.clbk_active, 10)
        
        #publisher til controller på /go_to_point/reached når målet er nådd
        self.reached_pub = self.create_publisher(Bool, '/go_to_point/reached', 10)

        # Statevariables
        self.active = False #aktiv gotopoint eller ikke
        self.target = None  #target, hvor vi skal
        self.position = None #position, hvor vi er
        self.yaw = None #retning tb peker
        self.reached_goal = True #Starter som nådd mål, venter på mål

        # Params
        self.heading_eps = 0.12 #hvor direkte vi må peke mot målet før vi kjører frem
        self.dist_eps = 0.12  #hvor close vi kan være målet før vi sier at det er nærme nok
        self.angular_speed = 0.8 # hvor raskt vi ønsker å kunne snu
        self.linear_speed = 0.35 # hvor raskt vi ønsker å kjøre fremover

    #funksjon for å sette active, aktiveres eller deaktiveres av bug2controlleren
    def clbk_active(self, msg: Bool):
        self.active = msg.data
        if not self.active:
            self._stop()
            self.get_logger().info("GTP-node deaktivert.")
        else:
            self.get_logger().info("GTP-node aktivert.")

    #funksjon som mottar mål fra rviz
    def clbk_goal(self, msg: PoseStamped):
        self.target = msg.pose.position
        self.reached_goal = False
      
        self.reached_pub.publish(Bool(data=False))
        self.get_logger().info(f"Posisjon: x={self.target.x:.2f}, y={self.target.y:.2f}")

    #funskjon for å hente posisjon og retning fra hjuldata (odom).
    def clbk_odom(self, msg: Odometry):
        self.position = msg.pose.pose.position
        q = msg.pose.pose.orientation #konvertering av quaternion
        eul = euler_from_quaternion((q.x, q.y, q.z, q.w)) #får yaw
        self.yaw = eul[2]
        self.navigate()

    #static metode for å kontrollere at vinkler alltid ligger mellom pos og neg pi.
    @staticmethod
    def _wrap(a):
        while a > math.pi:
            a -= 2 * math.pi
        while a < -math.pi:
            a += 2 * math.pi
        return a

    #funksjon for å stoppe roboten. publisher null hastighet.
    def _stop(self):
        self.cmd_pub.publish(Twist())

    #hovedfunksjon som gjør at tb kjører mot målet
    def navigate(self):
        #sjekk for at alt er good før vi kjører kode
        if not self.active or self.position is None or self.yaw is None or self.target is None or self.reached_goal:
            return

        #beregning av mål iforhold til posisjonen til roboten
        dx = self.target.x - self.position.x
        dy = self.target.y - self.position.y
        target_angle = math.atan2(dy, dx) #retning mot målet theta
        heading_error = self._wrap(target_angle - self.yaw) #hvor mye roboten peker feil
        distance = math.hypot(dx, dy) #hvor langt unna målet

        cmd = Twist() #cmd inneholder fart og rotasjon
        
        #peker vi for langt unna målet snur vi først
        if abs(heading_error) > self.heading_eps:
         
            cmd.angular.z = max(-self.angular_speed, min(self.angular_speed, 1.5 * heading_error))
        
        #kjør fremover hvis vi peker riktig vei
        elif distance > self.dist_eps:
            cmd.linear.x = self.linear_speed
        
        #stopp hvis vi er nære nok målet og logg "mål nådd"
        else:
            self.reached_goal = True
            self._stop()
        
            self.reached_pub.publish(Bool(data=True))
            self.get_logger().info("Mål nådd.")
            return 

        self.cmd_pub.publish(cmd) #send til robot via cmd

#starter noden
def main(args=None):
    rclpy.init(args=args)
    node = GoToPoint()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
