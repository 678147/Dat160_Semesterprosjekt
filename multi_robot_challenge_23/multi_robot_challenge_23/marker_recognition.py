import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

from geometry_msgs.msg import PoseArray, PoseStamped, Quaternion
from std_msgs.msg import Int32MultiArray, Bool
import tf2_ros
from tf_transformations import quaternion_from_euler

class MarkerFollower(Node):
    
    #Node som følger aruco-markør. Ser på hvor markøren er, finner avstanden og sier hvor roboten skal kjøre
    def __init__(self):
        super().__init__('marker_recognition')

        # parametre
        self.declare_parameter('robot_name', 'tb3_0')          # namespace på roboten
        self.declare_parameter('target_id', -1)                # -1, følg nærmeste av alle detekterte
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('aruco_prefix', 'aruco_')
        self.declare_parameter('standoff', 0.5)                # meter foran markøren (stoppeavstand)
        self.declare_parameter('min_reobserve', 2)             # hvor mange ganger vi må se samme markør før vi stoler på det
        self.declare_parameter('goal_timeout_sec', 10.0)       # hvor ofte vi kan sende nytt mål
        self.declare_parameter('pose_topic', 'aruco/poses')    #topic for posisjon til markers
        self.declare_parameter('ids_topic', 'aruco/ids')       #topic for ID til markers

        #tildeler verdier
        self.robot_name   = self.get_parameter('robot_name').get_parameter_value().string_value 
        self.target_id    = int(self.get_parameter('target_id').value)
        self.map_frame    = self.get_parameter('map_frame').get_parameter_value().string_value
        self.base_frame   = self.get_parameter('base_frame').get_parameter_value().string_value
        self.aruco_prefix = self.get_parameter('aruco_prefix').get_parameter_value().string_value
        self.standoff     = float(self.get_parameter('standoff').value)
        self.min_reobserve= int(self.get_parameter('min_reobserve').value)
        self.goal_timeout = float(self.get_parameter('goal_timeout_sec').value)
        self.pose_topic   = self.get_parameter('pose_topic').get_parameter_value().string_value
        self.ids_topic    = self.get_parameter('ids_topic').get_parameter_value().string_value

        #finner ut hvor ting er i forhold til kartet
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        #publishers og subscribers
        ns = f'/{self.robot_name}' #namespace
        self.pub_goal  = self.create_publisher(PoseStamped, f'{ns}/move_base_simple/goal', 10) #sender target til /move_base_simple/goal, der vi skal
        self.pub_found = self.create_publisher(Bool, f'{ns}/found_object', 10) #Sender True når roboten har funnet en markør (slik at andre noder vet)
        self.pub_active= self.create_publisher(Bool, f'{ns}/go_to_point/active', 10) #forteller /go_to_point/active at roboten nå skal begynne å gå mot et punkt

        self.sub_ids   = self.create_subscription(Int32MultiArray, self.ids_topic, self.on_ids, 10) #lytter på ID-nummer
        self.sub_poses = self.create_subscription(PoseArray, self.pose_topic, self.on_poses, 10) #Lytter etter posisjonene til markørene (hvor de er)

        self.last_ids = [] #siste ID-ene vi ser
        self.stable_counter = 0 #antall ganger sett en marker
        self.current_target_id: Optional[int] = None #ID-en vi følger
        self.last_goal_time: Optional[Time] = None #Tidspunkt for sist vi sendte mål
        self.have_announced_found = False #annonseringa av funnet

        self.get_logger().info(f"Marker recognition for {self.robot_name} er klar.")

    #Når roboten får en melding med ID-er
    def on_ids(self, msg: Int32MultiArray):
        self.last_ids = list(msg.data) #Lagrer ID-ene

    #funksjon som kjører når robotene får melding med posisjon fra markørene
    def on_poses(self, msg: PoseArray):
        if not self.last_ids or len(self.last_ids) != len(msg.poses):
            # IDs og poseliste må samsvare
            return

        # Velg markør-ID hvis vi følger en spesifikk ID
        if self.target_id >= 0:
            candidates = [(i, mid) for i, mid in enumerate(self.last_ids) if mid == self.target_id]
        else:
            # Vurder andre markører
            candidates = [(i, mid) for i, mid in enumerate(self.last_ids)]

        #hvis vi ikke har noen kanditater å følge
        if not candidates:
            self.stable_counter = 0
            self.current_target_id = None
            return

        #finn den nærmeste markøren
        chosen_index, chosen_id = candidates[0]
        min_dist = float('inf')
        now = self.get_clock().now()

        for i, mid in candidates:
            frame = f"{self.aruco_prefix}{mid}"
            try:
                tf = self.tf_buffer.lookup_transform(self.map_frame, frame, rclpy.time.Time())
                dx = tf.transform.translation.x
                dy = tf.transform.translation.y
                dist = math.hypot(dx, dy)
                if dist < min_dist:
                    min_dist = dist
                    chosen_index, chosen_id = i, mid
            except Exception:
                # hopp over hvis vi ikke finner posisjon
                pass

        # sjekk at vi har sett samme markør flere ganger (for å være sikker på at det er markør)
        if self.current_target_id == chosen_id:
            self.stable_counter += 1
        else:
            self.current_target_id = chosen_id
            self.stable_counter = 1

        if self.stable_counter < self.min_reobserve:
            return

        # Rate limit for å unngå spam av mål
        if self.last_goal_time is not None:
            if (now - self.last_goal_time) < Duration(seconds=self.goal_timeout):
                return

        # Slå opp markørens posisjon i map (x,y)
        frame = f"{self.aruco_prefix}{chosen_id}"
        try:
            tf_marker = self.tf_buffer.lookup_transform(self.map_frame, frame, rclpy.time.Time())
            mx = tf_marker.transform.translation.x
            my = tf_marker.transform.translation.y
        except Exception as e:
            self.get_logger().warn(f"Fant ikke TF til {frame}: {e}")
            return

        # Finn robotens posisjon i map for å beregne standoff og yaw mot markør
        try:
            tf_robot = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, rclpy.time.Time())
            rx = tf_robot.transform.translation.x
            ry = tf_robot.transform.translation.y
        except Exception as e:
            self.get_logger().warn(f"Fant ikke TF til {self.base_frame}: {e}")
            return

        # Vektor fra markør til robot (for å stoppe litt foran markøren)
        vx = rx - mx
        vy = ry - my
        norm = math.hypot(vx, vy)
        if norm < 1e-3:
            vx, vy = 1.0, 0.0
            norm = 1.0
        vx /= norm
        vy /= norm

        # Målposisjon i map med standoff (stopp 0.5m unna)
        gx = mx + vx * self.standoff
        gy = my + vy * self.standoff

        # Orienter roboten mot markøren
        yaw = math.atan2(my - gy, mx - gx)
        q = quaternion_from_euler(0.0, 0.0, yaw)
        
        #lager melding som sier hvor roboten skal kjøre (PoseStamped)
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = self.map_frame
        goal.pose.position.x = gx
        goal.pose.position.y = gy
        goal.pose.orientation = Quaternion(x=float(q[0]), y=float(q[1]), z=float(q[2]), w=float(q[3]))

        #si fra at objekt er funnet (pauser søket i Coordinator)
        if not self.have_announced_found:
            self.pub_found.publish(Bool(data=True))
            self.have_announced_found = True

        #aktiver lokal gtp
        self.pub_active.publish(Bool(data=True))

        #send mål til move_base_simple
        self.pub_goal.publish(goal)
        self.last_goal_time = now

        self.get_logger().info(
            f"[{self.robot_name}] ArUco {chosen_id} i map: "
            f"goal=({gx:.2f},{gy:.2f}) standoff={self.standoff:.2f} m"
        )


def main(args=None):
    rclpy.init(args=args)
    node = MarkerFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
