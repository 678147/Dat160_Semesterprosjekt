import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray, Pose, TransformStamped
from std_msgs.msg import Int32MultiArray
from cv_bridge import CvBridge
import numpy as np
import cv2
import tf2_ros

#node som gjenkjenner arucomarkøre i kameraet og skriver ut hvilket det tilsvarer.
#Deteketerer aruco markører med OpenCV
class ArucoNode(Node):
    
    #Flyt: Mottar kamerabilder og kamerainformasjon -> Finner hjørnene til Aruco-markører og leser ID -> 
    # Beregner posisjon og pose for hver markør m/kamerakalibrering -> publisher ID-er og pose
    
    
    def __init__(self):
        super().__init__('aruco_node') #navngir noden
       
       #parametere
        self.declare_parameter('marker_size', 0.05)       #størrelsen på markøren i meter          
        self.declare_parameter('aruco_dictionary_id', 'DICT_5X5_250') #aruco dictionary som brukes, satt fra lærer
        
        #ROS-topics vi lytter på for bilde og kamerainfo
        self.declare_parameter('image_topic', 'camera/image_raw') #her vi får bildene fra
        self.declare_parameter('camera_info_topic', 'camera/camera_info') #her vi får kamerainfo
        
        #Sender TF-ramme som inneholder posisjon og orientering
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('frame_prefix', 'aruco_')
        
        #Publisher debug bilde
        self.declare_parameter('publish_debug_image', True)

        #leser parameterverdier
        self.marker_size = float(self.get_parameter('marker_size').value)
        dict_name = str(self.get_parameter('aruco_dictionary_id').value)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.frame_prefix = str(self.get_parameter('frame_prefix').value)
        #self.publish_debug = bool(self.get_parameter('publish_debug_image').value)

        #Definerer hvilket sett med markører vi forventer (størrelse på mønster og antall ID-er)
        try:
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dict_name))
        except Exception as e:
            self.get_logger().error(f'Ukjent dict.: "{dict_name}": {e}')
            raise
        
        #deteksjonsparametere, standard
        self.aruco_params = cv2.aruco.DetectorParameters_create()

        #Konvertere mellom ROS-image og opencv bilder
        self.bridge = CvBridge()
        
        #kamerakalibrering (oppdateres når vi mottar CameraInfo)
        self.camera_matrix = None
        self.dist_coeffs = None

        #Topics vi lytter på for data
        img_topic = str(self.get_parameter('image_topic').value)
        cinfo_topic = str(self.get_parameter('camera_info_topic').value)

        #abonnerer på bilde og kamerainfo, QoS 10 for sensordata
        self.sub_img = self.create_subscription(Image, img_topic, self.on_image, 10)
        self.sub_cinfo = self.create_subscription(CameraInfo, cinfo_topic, self.on_camerainfo, 10)

        #Publishers id på aruco/ids, pose på aruco/poses,
        self.pub_ids = self.create_publisher(Int32MultiArray, 'aruco/ids', 10)
        self.pub_poses = self.create_publisher(PoseArray, 'aruco/poses', 10)
        #self.pub_dbg  = self.create_publisher(Image, 'aruco/debug_image', 10)

        #TF-broadcast for å legge markørrammer inn i TF
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        #logg for å vise at noden er klar
        self.get_logger().info('aruco_node klar.')

    #Callback-funksjon som mottar kamerakalibrering og lagrer K og D slik at vi kan beregne posen.
    def on_camerainfo(self, msg: CameraInfo):
        self.camera_matrix = np.array(msg.k, dtype=np.float32).reshape(3, 3)
        if len(msg.d) >= 4:
            self.dist_coeffs = np.array(msg.d, dtype=np.float32).reshape(-1)
        else:
            self.dist_coeffs = np.zeros((5,), dtype=np.float32)

    #funksjon som kjøres for hvert nytt kamerabilde
    def on_image(self, msg: Image):
        
        #gjør om bildet til OpenCV format med BGR-farger
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        #Finner aruco-markører
        corners, ids, _ = cv2.aruco.detectMarkers(frame, self.aruco_dict, parameters=self.aruco_params)
        
        # Hvis vi ikke fant noen markører i dette bildet, stopp her
        # (ellers ville ids vært None og ids.flatten() gitt feil)
        if ids is None or len(ids) == 0:
            return

        # Publisher ID-ene som ble funnet (overvåkes i RQT)
        id_msg = Int32MultiArray()
        id_msg.data = [int(i) for i in ids.flatten()]
        self.pub_ids.publish(id_msg)

        #Hvis vi ikke har fått kamerakalibrering enda, kan vi ikke gjøre noe videre
        if self.camera_matrix is None or self.dist_coeffs is None:
            return

        #Regner ut posen til hver markør sett fra kameraet.
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, self.marker_size, self.camera_matrix, self.dist_coeffs
        )

        # Lager en melding som inneholder posen for bildet
        pose_array = PoseArray()
        pose_array.header = msg.header #Tar med tidsstempel og hvilken ramme bildet tilhører
        
        for (rvec, tvec, mid) in zip(rvecs, tvecs, ids.flatten()):
            pose = Pose()
            
            #Posisjon i meter
            pose.position.x = float(tvec[0][0])
            pose.position.y = float(tvec[0][1])
            pose.position.z = float(tvec[0][2])

            # Gjør om rotasjonen til kvaternion (for ROS)
            R, _ = cv2.Rodrigues(rvec[0]) #3x3 rotasjonsmatrise
            qx, qy, qz, qw = self._rot_to_quat(R) #matrise til kvaternion (x,y,z,w)
            pose.orientation.x = qx
            pose.orientation.y = qy
            pose.orientation.z = qz
            pose.orientation.w = qw
            
            #legger posen inn i lista som publiseres på aruco/poses
            pose_array.poses.append(pose)

            #Sender TF ramme slik at systemet kan slå opp hvor markøren er
            if self.publish_tf:
                tf = TransformStamped()
                tf.header = msg.header       
                tf.child_frame_id = f'{self.frame_prefix}{int(mid)}'
                tf.transform.translation.x = pose.position.x
                tf.transform.translation.y = pose.position.y
                tf.transform.translation.z = pose.position.z
                tf.transform.rotation.x = qx
                tf.transform.rotation.y = qy
                tf.transform.rotation.z = qz
                tf.transform.rotation.w = qw
                self.tf_broadcaster.sendTransform(tf)

        #Publiser alle posene samlet
        self.pub_poses.publish(pose_array)

   #hjelpefunksjon som gjør om en 3x3 rotasjonsmatrise til kvaternion (x,y,z,w)
    @staticmethod
    def _rot_to_quat(R):
        t = np.trace(R)
        qw = np.sqrt(max(0.0, 1.0 + t)) / 2.0
        qx = (R[2, 1] - R[1, 2]) / (4.0 * qw + 1e-9)
        qy = (R[0, 2] - R[2, 0]) / (4.0 * qw + 1e-9)
        qz = (R[1, 0] - R[0, 1]) / (4.0 * qw + 1e-9)
        return float(qx), float(qy), float(qz), float(qw)


#lager noden
def main(args=None):
    rclpy.init(args=args)
    node = ArucoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
