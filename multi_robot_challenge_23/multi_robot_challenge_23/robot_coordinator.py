import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from rclpy.action import ActionClient
from mrc_interfaces.action import Navigate
from std_msgs.msg import String

import math 

class Coordinator(Node):
    def __init__(self):
        super().__init__('robot_coordinator')

        self.robots = ['robot1', 'robot2']
        self.clients = {}

        self.current_goals = {robot: None for robot in self.robots}
        self.robot_status = {robot: 'idle' for robot in self.robots}

        self.tasks = []


        for robot in self.robots:
            self.clients[robot] = ActionClient(self, Navigate, f'/{robot}/navigate')

            self.create_subscription(String, f'/{robot}/status', self.create_status_callback(robot), 10)
        

        self.create_timer(1.0, self.assign_goals)
        self.get_logger().info('Coordinator initialized.')
    
    def create_status_callback(self, robot_name):
        def callback(msg):
            self.robot_status[robot_name] = msg.data
            if msg.data.startswith("done"):
                self.current_goals[robot_name] = None
                self.robot_status[robot_name] = 'idle'
        return callback
    
    
    def status_callback(self, robot_name):
        def callback(msg):
            self.robot_status[robot_name] = msg.data
            if msg.data.startswith("task:"):
                _, coords = msg.data.split(":")
                x, y = map(float, coords.split(","))
                self.tasks_qeue.append(( x, y))
        return callback


    
    def send_goal(self, robot, goal_coords):
        x, y = goal_coords
        goal_msg = Navigate.Goal()
        goal_msg.x = x
        goal_msg.y = y

        client = self.clients[robot]
        
        future = client.send_goal_async(goal_msg)
    
    def assign_goals(self):
        while self.tasks_



        
