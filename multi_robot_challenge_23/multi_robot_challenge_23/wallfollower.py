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

        #publisher kjørekommandoer til turtleboten fart + rotasjon
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        #leser avstand fra LIDAR
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.clbk_scan, qos_profile_sensor_data)

        #Service som skrur wallfollower av og på i bug2controller er clienten.
        self.srv = self.create_service(SetBool, 'wall_follow', self.handle_srv)

        #parametre
        self.following = False   #starter med false, wallfollower ikke på
        self.wall_distance = 0.5 #ønsket avstand til vegg på høyre side
        self.front_threshold = 0.6 #hvor nær hindring foran før vi svinger unna
        self.turn_speed = 0.45 #hvor fort vi svinger
        self.forward_speed = 0.29 #fart fremover
        self.turning_counter = 0 #teller for å unngå å svinge for lenge til høyre

    #funksjon for å slå wallfollower av og på, får fra bug2 service client som gir boolsk verdi
    def handle_srv(self, req, res):
        self.following = req.data
        if not self.following:
            self.cmd_pub.publish(Twist()) 
        res.success = True
        res.message = 'WallFollower nå startet' if self.following else 'WallFollower er IKKE aktiv'
        return res

    #funksjon, statisk, for å konvertere en vinkel i radianer til en indeks i laserscan.ranges
    @staticmethod
    def _idx_for_angle(msg: LaserScan, ang: float) -> int:
        
        return int(round((ang - msg.angle_min) / msg.angle_increment))

    #funksjon for å hente vinkel-sektor rundt ang_center og returnerer alle finite målinger i vinduet
    #dette brukes for filtrere ut laserverdier rett foran og til høyre, +- 20grader
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

    #Funksjonen som leser laser og bestemmer hva wallfolloweren skal gjøre.
    def clbk_scan(self, msg: LaserScan):
        #ikke gjør noenting hvis wallfollower er av
        if not self.following:
            return

        #plukk ut to sektorer foran og til høyre
        front = self._window(msg, 0.0, math.radians(30))          
        right = self._window(msg, -math.pi / 2, math.radians(40)) 

        #hent nærmeste avstand i hver sektor
        front_min = min(front) if front else float('inf')
        right_min = min(right) if right else float('inf')

        cmd = Twist()

        #hindring rett foran? stopp og sving VENSTRE
        if front_min < self.front_threshold:
            
            cmd.linear.x = 0.0
            cmd.angular.z = self.turn_speed
            self.turning_counter = 0
        
        #for nær veggen på høyre side? trekk litt VENSTRE mens vi kjører
        elif right_min < self.wall_distance:
            
            cmd.linear.x = self.forward_speed
            cmd.angular.z = 0.1
            self.turning_counter = 0
        
        #For langt fra veggen til høyre? Sving litt til HØYRE i en periode
        elif right_min > self.wall_distance:
            
            #pulser mot høyre for å hente inn veggen igjen
            if self.turning_counter < 5:
                cmd.linear.x = 0.2
                cmd.angular.z = -self.turn_speed
                self.turning_counter += 1
            else:
                self.turning_counter = 0
        
        #kjør rett frem hvis i perfekt spot
        else:
        
            cmd.linear.x = self.forward_speed
            cmd.angular.z = 0.0

        #send til turtlebot
        self.cmd_pub.publish(cmd)

#start wallfollower noden
def main(args=None):
    rclpy.init(args=args)
    node = WallFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
