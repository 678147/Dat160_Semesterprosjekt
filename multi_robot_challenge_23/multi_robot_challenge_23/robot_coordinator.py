import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

class Coordinator(Node):
    def __init__(self):
        super().__init__('robot_coordinator')

        self.robots = ['tb3_0', 'tb3_1']

        # Publishers for goals
        self.goal_pubs = {r: self.create_publisher(PoseStamped,
                                                   f'/{r}/move_base_simple/goal',
                                                   QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
                          for r in self.robots}

        # Robot status and odom
        self.robot_status = {r: 'idle' for r in self.robots}
        self.robot_pose = {r: None for r in self.robots}
        self.assigned = {r: None for r in self.robots}

        for r in self.robots:
            self.create_subscription(String, f'/{r}/status', self._make_status_cb(r), 10)
            self.create_subscription(Odometry, f'/{r}/odom', self._make_odom_cb(r), 10)

        # Map subscription
        self.declare_parameter('map_topic', '/map')
        map_topic = self.get_parameter('map_topic').get_parameter_value().string_value

        map_qos = QoSProfile(depth=1)
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(OccupancyGrid, map_topic, self.map_cb, map_qos)

        # Frontier visualization
        frontier_qos = QoSProfile(depth=1)
        frontier_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.frontier_pub = self.create_publisher(MarkerArray, '/frontier_markers', frontier_qos)

        self.map = None
        self.frontiers = []

        self.create_timer(1.0, self.assign_goals)
        self.get_logger().info('Coordinator initialized.')

    def _make_status_cb(self, robot):
        def cb(msg: String):
            self.robot_status[robot] = msg.data
            if msg.data == 'done':
                self.assigned[robot] = None
        return cb

    def _make_odom_cb(self, robot):
        def cb(msg: Odometry):
            self.robot_pose[robot] = msg.pose.pose
        return cb

    def map_cb(self, msg: OccupancyGrid):
        self.map = msg
        # Compute frontiers **per robot half**
        frontiers = []
        for i, r in enumerate(self.robots):
            half_map = self._mask_map_half_for_robot(msg, i)
            frontiers += self._compute_frontiers(half_map)
        self.frontiers = frontiers

        # Publish markers
        ma = MarkerArray()
        for i, (cx, cy) in enumerate(self.frontiers):
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'frontiers'
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = float(cx)
            m.pose.position.y = float(cy)
            m.pose.position.z = 0.1
            m.scale.x = 0.2
            m.scale.y = 0.2
            m.scale.z = 0.2
            m.color.a = 1.0
            m.color.r = 1.0
            m.color.g = 0.5
            m.color.b = 0.0
            ma.markers.append(m)
        self.frontier_pub.publish(ma)

    def _mask_map_half_for_robot(self, grid: OccupancyGrid, robot_index: int, free_radius_m: float = 1.0) -> OccupancyGrid:
        """Return a copy of the map where only the robot's half is considered for exploration."""
        w = grid.info.width
        h = grid.info.height
        res = grid.info.resolution
        ox = grid.info.origin.position.x
        oy = grid.info.origin.position.y

        data = [-1] * (w * h)  # unknown everywhere

        # Vertical split
        start_x = int((w / len(self.robots)) * robot_index)
        end_x = int((w / len(self.robots)) * (robot_index + 1))

        r_name = self.robots[robot_index]
        pose = self.robot_pose.get(r_name)
        if pose:
            cx = int((pose.position.x - ox) / res)
            cy = int((pose.position.y - oy) / res)
            cell_radius = max(1, int(math.ceil(free_radius_m / res)))
            for dy in range(-cell_radius, cell_radius + 1):
                y = cy + dy
                if y < 0 or y >= h:
                    continue
                for dx in range(-cell_radius, cell_radius + 1):
                    x = cx + dx
                    if x < start_x or x >= end_x or x >= w:
                        continue
                    idx = y * w + x
                    data[idx] = 0  # free

        new_grid = OccupancyGrid()
        new_grid.header = grid.header
        new_grid.info = grid.info
        new_grid.data = data
        return new_grid

    def _compute_frontiers(self, grid: OccupancyGrid):
        w = grid.info.width
        h = grid.info.height
        data = grid.data

        def idx(x, y):
            return y * w + x

        frontier_cells = set()
        for y in range(h):
            for x in range(w):
                if data[idx(x, y)] != 0:
                    continue
                for dx, dy in ((1,0),(-1,0),(0,1),(0,-1)):
                    nx, ny = x+dx, y+dy
                    if 0 <= nx < w and 0 <= ny < h and data[idx(nx, ny)] == -1:
                        frontier_cells.add((x, y))
                        break

        visited = set()
        clusters = []
        for cell in frontier_cells:
            if cell in visited:
                continue
            stack = [cell]
            comp = []
            while stack:
                c = stack.pop()
                if c in visited:
                    continue
                visited.add(c)
                comp.append(c)
                x, y = c
                for dx, dy in ((1,0),(-1,0),(0,1),(0,-1)):
                    n = (x+dx, y+dy)
                    if n in frontier_cells and n not in visited:
                        stack.append(n)
            clusters.append(comp)

        goals = []
        origin = grid.info.origin
        res = grid.info.resolution
        for comp in clusters:
            xs = [origin.position.x + (x + 0.5) * res for x, _ in comp]
            ys = [origin.position.y + (y + 0.5) * res for _, y in comp]
            cx = sum(xs) / len(xs)
            cy = sum(ys) / len(ys)
            goals.append((cx, cy))

        return goals

    def assign_goals(self):
        if not self.map:
            return
        if not self.frontiers:
            return

        min_dist_to_robot = 0.3  # minimum distance from robot to frontier

        # build list of available frontiers not already assigned
        free_goals = [g for g in self.frontiers if g not in list(self.assigned.values())]

        if not free_goals:
            return

        # For each robot that is idle and has no assignment
        for r in self.robots:
            if self.assigned[r] is not None:
                continue
            if self.robot_status.get(r, 'idle') != 'idle':
                continue

            pose = self.robot_pose.get(r)
            if pose is None:
                continue  # can't compute distance without pose

            # Split map per robot: simple example using y-axis
            robot_y = pose.position.y
            robot_half = 'bottom' if robot_y < 0 else 'top'

            # filter free goals by distance and half-map
            filtered_goals = []
            for g in free_goals:
                d = math.hypot(g[0] - pose.position.x, g[1] - robot_y)
                if d < min_dist_to_robot:
                    continue

                # half-map filter
                if robot_half == 'bottom' and g[1] > 0:
                    continue
                if robot_half == 'left' and g[0] < 0:
                    continue

                filtered_goals.append(g)

            if not filtered_goals:
                continue

            # choose nearest goal
            best = min(filtered_goals, key=lambda g: math.hypot(g[0]-pose.position.x, g[1]-pose.position.y))

            # build PoseStamped
            ps = PoseStamped()
            ps.header.frame_id = 'map'
            ps.header.stamp = self.get_clock().now().to_msg()
            from geometry_msgs.msg import Pose, Point, Quaternion
            ps.pose = Pose()
            ps.pose.position = Point(x=best[0], y=best[1], z=0.0)
            ps.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

            # publish goal
            self.goal_pubs[r].publish(ps)
            self.assigned[r] = best

            # mark goal taken
            try:
                free_goals.remove(best)
            except ValueError:
                pass

            self.get_logger().info(f'Assigned goal ({best[0]:.2f},{best[1]:.2f}) to {r}')


def main(args=None):
    rclpy.init(args=args)
    node = Coordinator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
