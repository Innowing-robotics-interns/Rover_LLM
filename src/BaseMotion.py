import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PointStamped
from utils import to_euler
import math


# forward (+ve) and backward 
def go_Xaxis(cmd, position_receiver): 
    dis   = cmd[0]
    point = []
    rclpy.spin_once(position_receiver)
    x = position_receiver.current_pose.x
    y = position_receiver.current_pose.y
    z = position_receiver.current_pose.z
    rx = position_receiver.current_orientation.x
    ry = position_receiver.current_orientation.y
    rz = position_receiver.current_orientation.z
    rw = position_receiver.current_orientation.w
    position = [x, y, z]
    rotation = [rx, ry, rz, rw]
    euler = to_euler(rotation)
    position[0] += dis*math.cos(euler[2])
    position[1] += dis*math.sin(euler[2])
    point.append(position)
    return point


# left (+ve) and right
def go_Yaxis(cmd, position_receiver):
    dis   = cmd[0]
    point = []
    rclpy.spin_once(position_receiver)
    x = position_receiver.current_pose.x
    y = position_receiver.current_pose.y
    z = position_receiver.current_pose.z
    rx = position_receiver.current_orientation.x
    ry = position_receiver.current_orientation.y
    rz = position_receiver.current_orientation.z
    rw = position_receiver.current_orientation.w
    position = [x, y, z]
    rotation = [rx, ry, rz, rw]
    euler = to_euler(rotation)
    euler[2] += 90
    position[0] += dis*math.cos(euler[2])
    position[1] += dis*math.sin(euler[2])
    position_receiver.publish_point(position)
    point.append(position)
    return point

# def go_right(cmd, position_receiver):
#     dis   = cmd[0]
#     point = []
#     rclpy.spin_once(position_receiver)
#     x = position_receiver.current_pose.x
#     y = position_receiver.current_pose.y
#     z = position_receiver.current_pose.z
#     rx = position_receiver.current_orientation.x
#     ry = position_receiver.current_orientation.y
#     rz = position_receiver.current_orientation.z
#     rw = position_receiver.current_orientation.w
#     position = [x, y, z]
#     rotation = [rx, ry, rz, rw]
#     euler = to_euler(rotation)
#     euler[2] -= 90
#     position[0] += dis*math.cos(euler[2])
#     position[1] += dis*math.sin(euler[2])
#     position_receiver.publish_point(position)
#     point.append(position)
#     return point

def go_to_point(cmd, position_receiver):
    x_t   = cmd[0]
    y_t   = cmd[1]
    point = []
    rclpy.spin_once(position_receiver)
    # x = position_receiver.current_pose.x
    # y = position_receiver.current_pose.y
    # z = position_receiver.current_pose.z
    # rx = position_receiver.current_orientation.x
    # ry = position_receiver.current_orientation.y
    # rz = position_receiver.current_orientation.z
    # rw = position_receiver.current_orientation.w
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

# draw a circl with the current position as a point above the circle
def ego_circle(cmd, position_receiver):
    radius = cmd[0]
    point  = []
    rclpy.spin_once(position_receiver)
    x = position_receiver.current_pose.x
    y = position_receiver.current_pose.y
    z = position_receiver.current_pose.z
    position = [x, y, z]
    x_c = x-radius
    y_c = y
    for i in range(100):
        x_m = x_c + radius*math.cos(i/100*2*math.pi)
        y_m = y_c + radius*math.sin(i/100*2*math.pi)
        position = [x_m, y_m, 0]
        point.append(position)
    return point
