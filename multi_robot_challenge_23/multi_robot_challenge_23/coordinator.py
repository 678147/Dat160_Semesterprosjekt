import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
import random

class Coordinator(Node):
    def __init__(self):
        super().__init__('coordinator')

        self.robots = ['tb3_0', 'tb3_1']
        self.pause_timer = self.create_timer(2.0, self.check_resume)
        self.pause = 2.0
        self.pause_start_time = None
        # Publishers for goals and activation
        self.goal_pubs = {
            r: self.create_publisher(PoseStamped, f'/{r}/move_base_simple/goal', 10)
            for r in self.robots
        }
        self.active_pubs = {
            r: self.create_publisher(Bool, f'/{r}/go_to_point/active', 10)
            for r in self.robots
        }

        # Subscriptions for goal reached and object detection
        self.reached_subs = {
            r: self.create_subscription(
                Bool, f'/{r}/go_to_point/reached', lambda m, rr=r: self.on_reached(m, rr), 10
            )
            for r in self.robots
        }
        self.found_subs = {
            r: self.create_subscription(
                Bool, f'/{r}/found_object', lambda m, rr=r: self.on_found(m, rr), 10
            )
            for r in self.robots
        }

        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL  

        self.map_received = False
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.clbk_map, qos)

    
        self.map = None
        self.paths = {r: [] for r in self.robots}
        self.index = {r: 0 for r in self.robots}
        self.busy = {r: False for r in self.robots}
        self.paused = False

    
    def clbk_map(self, msg):
        if self.map_received:
            return  #Allerede prossesert kartet
        self.map_received = True
        self.map = msg
        self.get_logger().info("Map received. Coordinator ready to assign dynamic goals.")

        #første mål til alle roboter
        for r in self.robots:
            self.assign_goal(r)

    #gi mål til robot
    def assign_goal(self, robot):
        if not self.map or self.paused:
            return # kart ikke mottatt eller pauset 

        width = self.map.info.width
        height = self.map.info.height
        res = self.map.info.resolution
        origin = self.map.info.origin.position

        free_cells = [
            (x, y)
            for y in range(height)
            for x in range(width)
            if self.map.data[y * width + x] == 0
        ]

        if not free_cells:
            self.get_logger().warn("No free cells available!")
            return

        x_cell, y_cell = random.choice(free_cells)
        x = origin.x + x_cell * res
        y = origin.y + y_cell * res

        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.w = 1.0

        self.goal_pubs[robot].publish(goal)
        self.active_pubs[robot].publish(Bool(data=True))
        self.busy[robot] = True
        self.get_logger().info(f"Sent dynamic goal to {robot}: ({x:.2f}, {y:.2f})")

    #Robot nådd et mål
    def on_reached(self, msg, robot):
        if msg.data:
            self.busy[robot] = False
            self.get_logger().info(f"{robot} reached its goal. Assigning new goal...")
            self.assign_goal(robot)

    #Robot fant POI
    def on_found(self, msg, robot):
        if msg.data:
            self.get_logger().warn(f"[{robot}] Found object! Pausing search.")
            self.paused = True
            self.pause_start_time = self.get_clock().now().seconds_nanoseconds()[0]
            for r in self.robots:
                self.active_pubs[r].publish(Bool(data=False))

    #Finner søkepunkter i kartet
    def generate_search_points(self, occupancy_grid, step=0.5):
        data = occupancy_grid.data
        width = occupancy_grid.info.width
        height = occupancy_grid.info.height
        res = occupancy_grid.info.resolution
        origin = occupancy_grid.info.origin.position

        waypoints = []
        step_cells = max(1, int(step / res))

        for y in range(0, height, step_cells):
            row = []
            for x in range(0, width, step_cells):
                i = y * width + x
                if 0 <= i < len(data) and data[i] == 0:  # free space
                    wx = origin.x + x * res
                    wy = origin.y + y * res
                    row.append((wx, wy))
            if y % 2 == 0:
                waypoints.extend(row)
            else:
                waypoints.extend(row[::-1])

        random.shuffle(waypoints)
        return waypoints
    
    #GJennoptar søket etter pause
    def check_resume(self):
        if self.paused and self.pause_start_time is not None:
            now = self.get_clock().now().seconds_nanoseconds()[0] - self.pause_start_time
            if now >= self.pause:
                self.get_logger().info("Resuming search for all robots.")
                self.paused = False
                self.pause_start_time = None
                for r in self.robots:
                    if not self.busy[r]:
                        self.assign_goal(r)


def main(args=None):
    rclpy.init(args=args)
    node = Coordinator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
