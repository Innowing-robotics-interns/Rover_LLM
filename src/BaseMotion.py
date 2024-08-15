import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PointStamped
from utils import to_euler
import math

class Position_Receiver(Node):
    def __init__(self):
        super().__init__('position_receiver')
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )
        self.point_pub = self.create_publisher(
            PointStamped,
            'clicked_point',
            10
        )
        self.current_position = None
        self.current_orientation = None

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose.position
        self.current_orientation = msg.pose.pose.orientation

    def publish_point(self, target_point):
        point_msg = PointStamped()
        point_msg.point.x = target_point[0]
        point_msg.point.y = target_point[1]
        point_msg.point.z = target_point[2]
        self.point_pub.publish(point_msg)

# TODO: Implement the following class
class BASEMOTION:
    def __init__(self):
        self.position_receiver = Position_Receiver()
        self.position_receiver.current_pose = [0, 0, 0]
        self.position_receiver.current_orientation = [0, 0, 0, 0]
        self.position_now = [0, 0, 0]
        self.command_map = {
            0: lambda command: stop(command, self.position_receiver),
            1: lambda command: go_Xaxis(command, self.position_receiver),
            2: lambda command: go_Yaxis(command, self.position_receiver),
            3: lambda command: go_to_point(command, self.position_receiver),
            4: lambda command: circle(command, self.position_receiver),
            5: lambda command: setMark(command, self.position_receiver)
        }
        
    def cmd_processing(self, cmd):
        path = []
        list = eval(cmd)
        action_history = {0: ""}
        action_history_max_len = 20
        action_history_idx = 0
        print(list)
        for command in list:
            if command[0] in self.command_map:
                action_history_idx = (action_history_idx + 1) % action_history_max_len
                action_history[action_history_idx] = command
                if 0 < command[0] < 10:
                    path += [f'Planer({command})'] # temporary use for testing
                    # path += self.command_map[command[0]](command[1:])
        # entire action list should be collected here
        # error detect and handle can be added below
        print(path)
        if len(path) > 0:
            for target_point in path:
                rclpy.spin_once(self.position_receiver)
                self.position_now[0] = self.position_receiver.current_pose.x
                self.position_now[1] = self.position_receiver.current_pose.y
                self.position_now[2] = self.position_receiver.current_pose.z
                self.position_receiver.publish_point(target_point)
                print('send a target_point!')
                bias = ((self.position_now[0] - target_point[0])**2 + (self.position_now[1] - target_point[1])**2 + (self.position_now[2] - target_point[2])**2)**0.5
                while bias > 0.01:
                    rclpy.spin_once(self.position_receiver)
                    #position_receiver.publish_point(target_point)
                    self.position_now[0] = self.position_receiver.current_pose.x
                    self.position_now[1] = self.position_receiver.current_pose.y
                    self.position_now[2] = self.position_receiver.current_pose.z
                    bias = ((self.position_now[0] - target_point[0])**2 + (self.position_now[1] - target_point[1])**2 + (self.position_now[2] - target_point[2])**2)**0.5

# forward (+ve) and backward 
def go_Xaxis(cmd, position_receiver): 
    dis          = cmd[0]
    point        = []
    rclpy.spin_once(position_receiver)
    x            = position_receiver.current_pose.x
    y            = position_receiver.current_pose.y
    z            = position_receiver.current_pose.z
    rx           = position_receiver.current_orientation.x
    ry           = position_receiver.current_orientation.y
    rz           = position_receiver.current_orientation.z
    rw           = position_receiver.current_orientation.w
    position     = [x, y, z]
    rotation     = [rx, ry, rz, rw]
    euler        = to_euler(rotation)
    position[0] += dis*math.cos(euler[2])
    position[1] += dis*math.sin(euler[2])
    point.append(position)
    return point


# left (+ve) and right
def go_Yaxis(cmd, position_receiver):
    dis          = cmd[0]
    point        = []
    rclpy.spin_once(position_receiver)
    x            = position_receiver.current_pose.x
    y            = position_receiver.current_pose.y
    z            = position_receiver.current_pose.z
    rx           = position_receiver.current_orientation.x
    ry           = position_receiver.current_orientation.y
    rz           = position_receiver.current_orientation.z
    rw           = position_receiver.current_orientation.w
    position     = [x, y, z]
    rotation     = [rx, ry, rz, rw]
    euler        = to_euler(rotation)
    euler[2]    += 90
    position[0] += dis*math.cos(euler[2])
    position[1] += dis*math.sin(euler[2])
    position_receiver.publish_point(position)
    point.append(position)
    return point


def go_to_point(cmd, position_receiver):
    x_t      = cmd[0]
    y_t      = cmd[1]
    point    = []
    position = [x_t, y_t, 0]
    position_receiver.publish_point(position)
    point.append(position)
    return point

# rotate around a given point with given radius
def circle(cmd, position_receiver):
    x_c    = cmd[0]
    y_c    = cmd[1]
    radius = cmd[2]
    point  = []
    for i in range(100):
        x = x_c + radius*math.cos(i/100*2*math.pi)
        y = y_c + radius*math.sin(i/100*2*math.pi)
        position = [x, y, 0]
        point.append(position)
    return point


def stop(cmd, position_receiver):
    point = []
    position_receiver.publish_point(position_receiver.current_pose)
    point.append(position_receiver.current_pose)
    return point

def setMark(cmd, position_receiver):
    name = cmd[0]
    position = [position_receiver.current_pose.x, position_receiver.current_pose.y, position_receiver.current_pose.z]
    return [name, position]