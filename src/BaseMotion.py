import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PointStamped, Twist, Point
from utils import to_euler, angle_calculation, value_CLIP, value_NORM
import math
from std_msgs.msg import Bool
import string

PI = math.pi
ANGLE_THRESHOLD = 0.09

class Position_Receiver(Node):
    def __init__(self):
        super().__init__('position_receiver')
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom_baselink',
            self.odom_callback,
            10
        )
        self.point_pub = self.create_publisher(
            PointStamped,
            'clicked_point',
            10
        )
        self.nav_reply_sub = self.create_subscription(
            Bool,
            'dstar_arrive',
            self.nav_reply_callback,
            10
        )
        self.vision_detect_sub = self.create_subscription(
            PointStamped,
            'detect_point',
            self.vision_detect_callback,
            10
        )
        
        self.last_pose= [0.0, 0.0, 0.0] #[x, y, theta]
        self.current_orientation = None
        self.LocationLibrary = {}
        self.facing_flag = False
        self.facing_point = None
        self.nav_arrived = False 
        self.delta_theta = 0.0
        self.vision_detect = [0.0, 0.0] # [x, y]
        self.watch_dog = 0

    def vision_detect_callback(self, msg):
        self.vision_detect = [msg.point.x, msg.point.y]

    def nav_reply_callback(self, msg):
        # self.get_logger().info(f'nav_reply: {msg.data}')
        self.watch_dog += 1
        if msg.data and self.watch_dog > 10:
            self.nav_arrived = True
            self.watch_dog = 0  
        else:
            self.nav_arrived = False
            

    def odom_callback(self, msg):
        # self.get_logger().info(f'position: {msg.pose.pose.position.x}, {msg.pose.pose.position.y}, {msg.pose.pose.position.z}, orientation: {msg.pose.pose.orientation.x}, {msg.pose.pose.orientation.y}, {msg.pose.pose.orientation.z}, {msg.pose.pose.orientation.w}')
        rotation = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        self.current_pose = [msg.pose.pose.position.x, msg.pose.pose.position.y, to_euler(rotation)[2]] # x, y, euler angle in z axis
                

    def publish_point(self, target_point):
        point_msg = PointStamped()
        timer_period = 0.5

        delta_theta = target_point[2] - self.current_pose[2]
        if delta_theta > PI:
            delta_theta -= 2*PI
        if delta_theta < -PI:
            delta_theta += 2*PI
        self.delta_theta = delta_theta
        wz = value_CLIP((delta_theta / timer_period), -0.6, 0.6)

        point_msg.point.x = target_point[0]
        point_msg.point.y = target_point[1]
        point_msg.point.z = wz
        
        self.point_pub.publish(point_msg)


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
            5: lambda command: setMark(command, self.position_receiver),
            6: lambda command: turn(command, self.position_receiver),
        }   

    def run(self, cmd):
        path = []
        cnt = 0
        cmd = eval(cmd)
        print("cmd: ",cmd)
        # single action
        for command in cmd:
            print("command: ",command)
            if command[0] in self.command_map:
                if 0 < command[0] < 10:
                    path = self.command_map[command[0]](command[1:])
                    if command[0] == 5 and path[0].lower() != 'ff':
                        print('set mark:', path)
                        self.position_receiver.LocationLibrary[path[0]] = path[1]
                        continue
            # print("path: ",path)
            if len(path) > 0:
                for target_point in path:
                    rclpy.spin_once(self.position_receiver)
                    self.position_receiver.last_pose = self.position_receiver.current_pose
                    self.position_receiver.nav_arrived = False
                    print("Ready to move...")
                    # continue # debug purpose
                    while (self.position_receiver.nav_arrived == False) or (self.position_receiver.delta_theta > ANGLE_THRESHOLD or self.position_receiver.delta_theta < -ANGLE_THRESHOLD):
                        # print("nav_arrived1: ", self.position_receiver.nav_arrived)
                        rclpy.spin_once(self.position_receiver)
                        self.pose_now = self.position_receiver.current_pose
                        # print("target: ", target_point, "current: ", self.pose_now)
                        # print("delta_theta: ", self.position_receiver.delta_theta)
                        # print("nav_arrived: ", self.position_receiver.nav_arrived)
                        # input()
                        self.position_receiver.publish_point(target_point)
                        
                    # if (self.position_receiver.nav_arrived == True):
                    #     cnt += 1
                    # print(cnt)
                    self.position_receiver.facing_flag = False
                    self.position_receiver.facing_point = None
                    self.position_receiver.nav_arrived = False

                        

# forward (+ve) and backward 
def go_Xaxis(cmd, cmd_node): 
    dis          = cmd[0]
    namef        = cmd[1].lower()
    x_f          = cmd[2]
    y_f          = cmd[3]
    point        = []
    rclpy.spin_once(cmd_node)
    position     = cmd_node.current_pose
    euler = position[2]
    if dis == '+FF':
        dis = 1
    elif dis == '-FF':
        dis = -1
    position[0] += dis*math.cos(euler)
    position[1] += dis*math.sin(euler)
    if namef in cmd_node.LocationLibrary:
        x_f      = cmd_node.LocationLibrary[namef][0]
        y_f      = cmd_node.LocationLibrary[namef][1]
        print("cmd_nodeLibrary: ",cmd_node.LocationLibrary)
    elif x_f == 'FF' and y_f == 'FF':
        x_f      = position[0]
        y_f      = position[1]
    # cmd_node.facing_flag = True
    # cmd_node.facing_point = [x_f, y_f]
    
    point.append(position)
    return point

# forward (+ve) and backward 
def go_Yaxis(cmd, cmd_node): 
    dis          = cmd[0]
    namef        = cmd[1].lower()
    x_f          = cmd[2]
    y_f          = cmd[3]
    point        = []
    rclpy.spin_once(cmd_node)
    position     = cmd_node.current_pose
    euler = position[2] + 90.0 / 180.0 * PI
    

    if dis == '+FF':
        dis = 1
    elif dis == '-FF':
        dis = -1
    position[0] += dis*math.cos(euler)
    position[1] += dis*math.sin(euler)
    if namef in cmd_node.LocationLibrary:
        x_f      = cmd_node.LocationLibrary[namef][0]
        y_f      = cmd_node.LocationLibrary[namef][1]
        print("cmd_nodeLibrary: ",cmd_node.LocationLibrary)
    elif x_f == 'FF' and y_f == 'FF':
        x_f      = position[0]
        y_f      = position[1]
    # cmd_node.facing_flag = True
    # cmd_node.facing_point = [x_f, y_f]
    
    point.append(position)
    return point

# [Name x, y, NameF, xf, yf]
def go_to_point(cmd, cmd_node):
    name         = cmd[0]
    x_t          = cmd[1]
    y_t          = cmd[2]
    namef        = cmd[3].lower()
    x_f          = cmd[4]
    y_f          = cmd[5]
    point        = []
    
    rclpy.spin_once(cmd_node)
    position     = cmd_node.current_pose
    euler = position[2]
    
    if name in cmd_node.LocationLibrary:
        x_t      = cmd_node.LocationLibrary[name][0]
        y_t      = cmd_node.LocationLibrary[name][1]
        print("cmd_nodeLibrary: ",cmd_node.LocationLibrary)
    elif 'last' in name:   
        x_t     = cmd_node.last_pose[0]
        y_t     = cmd_node.last_pose[1]
        print("last postion:", x_t, y_t)
    else:
        x_t     = cmd[1]
        y_t     = cmd[2]
    if x_t == 'FF' and y_t == 'FF':
        x_t     = position[0]
        y_t     = position[1]
    position    = [x_t, y_t, euler]
    
    if namef in cmd_node.LocationLibrary:
        x_f     = cmd_node.LocationLibrary[namef][0]
        y_f     = cmd_node.LocationLibrary[namef][1]
    elif x_f == 'FF' and y_f == 'FF':
        x_f     = position[0]
        y_f     = position[1]
    # cmd_node.facing_flag = True
    # cmd_node.facing_point = [x_f, y_f]
    point.append(position)
    return point

# rotate around a given point with given radius
# [4, Name, x, y, radius, number of circles, IsFacing, xf, yf]
def circle(cmd, cmd_node):
    name          = cmd[0].lower()
    x_c           = cmd[1]
    y_c           = cmd[2]
    radius        = cmd[3]
    IsFacing      = cmd[5]
    x_f           = cmd[6]
    y_f           = cmd[7]
    point         = []
    
    if name in cmd_node.LocationLibrary:
        x_c       = cmd_node.LocationLibrary[name][0]
        y_c       = cmd_node.LocationLibrary[name][1]
        print("cmd_nodeLibrary: ",cmd_node.LocationLibrary)
    elif 'vision' in name:
        x_c       = cmd_node.vision_detect[0]
        y_c       = cmd_node.vision_detect[1]
        print("vision target", x_c, y_c)
    elif x_c == 'FF' or y_c == 'FF':
        x_c       = cmd_node.current_pose[0]
        y_c       = cmd_node.current_pose[1]
    if radius == 'FF':
        radius    = 0.5
    if IsFacing and (x_f == 'FF' or y_f == 'FF'):
        x_f       = x_c
        y_f       = y_c
        
    rclpy.spin_once(cmd_node)
    position      = cmd_node.current_pose
    # position      = [0.0,0.0,0.0]


    for i in range(100):
        x         = x_c + radius*math.cos(i/100*2*PI)
        y         = y_c + radius*math.sin(i/100*2*PI)
        if IsFacing:
            euler = angle_calculation(x, y, x_f, y_f)
        else:   
            euler = position[2]
                    
        position  = [x, y, euler]
        point.append(position)
    return point

def stop(cmd, cmd_node):
    point = []
    rclpy.spin_once(cmd_node)
    position     = cmd_node.current_pose
    euler = position[2]
    
    position     = [position[0], position[1], euler]
    point.append(position)
    return point

def setMark(cmd, cmd_node):
    name = cmd[3].lower()
    rclpy.spin_once(cmd_node)
    position     = cmd_node.current_pose
    position     = [position[0], position[1]]
    return [name, position]

# [Name, x, y, direction, angle]
def turn(cmd, cmd_node):
    point = []
    rclpy.spin_once(cmd_node)
    name = cmd[0].lower()
    if name in cmd_node.LocationLibrary:
        x_t      = cmd_node.LocationLibrary[name][0]
        y_t      = cmd_node.LocationLibrary[name][1]
        print("cmd_nodeLibrary: ",cmd_node.LocationLibrary)
    elif cmd[1] == 'FF' or cmd[2] == 'FF':
        euler    = cmd[3] * cmd[4] / 180.0 * PI
        position = [cmd_node.current_pose[0], cmd_node.current_pose[1], cmd_node.current_pose[2] + euler]
        point.append(position)
        return point
    else:
        x_t      = cmd[1]
        y_t      = cmd[2]
    euler = angle_calculation(cmd_node.current_pose[0], cmd_node.current_pose[1], x_t, y_t)
    position = [cmd_node.current_pose[0], cmd_node.current_pose[1], euler]
    point.append(position)
    return point



# test
def main(args=None):
    rclpy.init(args=args)
    position_receiver = Position_Receiver()
    test = BASEMOTION()
    # rclpy.spin(position_receiver)
    test_path = [[2, 1, 'FF', 'FF', 'FF']]
    test.run(str(test_path))
    position_receiver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()