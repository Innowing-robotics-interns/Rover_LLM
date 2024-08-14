from LMP import LMP
from utils import get_config
from BaseMotion import go_Xaxis, go_Yaxis, go_to_point, circle, ego_circle
import numpy, subprocess, time

import os
import openai
from openai import AzureOpenAI
from dotenv import load_dotenv
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PointStamped


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

# idea adapted from THU students
def cmd_processing(cmd):
	path = []
	list = eval(cmd)
	print(list)
	for command in list:
		print(command)
		if command[0] in command_map:
			path += command_map[command[0]](command[1:])

	print(path)
	if len(path) > 0:
		for target_point in path:
			rclpy.spin_once(position_receiver)
			position_now[0] = position_receiver.current_pose.x
			position_now[1] = position_receiver.current_pose.y
			position_now[2] = position_receiver.current_pose.z
			position_receiver.publish_point(target_point)
			print('send a target_point!')
			bias = ((position_now[0] - target_point[0])**2 + (position_now[1] - target_point[1])**2 + (position_now[2] - target_point[2])**2)**0.5
			while bias > 0.01:
				rclpy.spin_once(position_receiver)
				#position_receiver.publish_point(target_point)
				position_now[0] = position_receiver.current_pose.x
				position_now[1] = position_receiver.current_pose.y
				position_now[2] = position_receiver.current_pose.z
				bias = ((position_now[0] - target_point[0])**2 + (position_now[1] - target_point[1])**2 + (position_now[2] - target_point[2])**2)**0.5
	path.clear()

def model_init():
	cfg = get_config('configs/config.yaml')['lmps']
	fixed_vars = {'numpy':numpy, 'subprocess':subprocess, 'time': time} # for third libraries that LLM can access
	variable_vars = {} # for first party libraries (can be other LLM) that a LLM can access
	# allow LMPs to access other LMPs
	# & low-level LLM setup
	lmp_names = [name for name in cfg.keys() if not name in ['coder','previewer']] # cfg=lmps_config
	low_level_lmps = {
		k: LMP(k, cfg[k], fixed_vars, variable_vars) #, debug, env_name)
		for k in lmp_names
	}
	variable_vars.update(low_level_lmps)

	# high-level LLM setup
	coder = LMP("coder", cfg['coder'], fixed_vars, variable_vars)
	previewer = LMP("previewer", cfg['previewer'])
 
	return previewer, coder

def main():
    # declare global variables
	global position_receiver, position_now, command_map
    # init
	rclpy.init()
	preview, model = model_init() 
	position_receiver = Position_Receiver()
	position_now = [0, 0, 0]
	command_map = {
        1: lambda command: go_Xaxis(command, position_receiver),
        2: lambda command: go_Yaxis(command, position_receiver),
        3: lambda command: go_to_point(command, position_receiver),
        4: lambda command: circle(command, position_receiver),
        5: lambda command: ego_circle(command, position_receiver)
    }

	# test
	# test_llm = LMP("test", get_config('configs/config.yaml')['lmps']['test'])
	# while True:
	# 	# if True:
	# 	input_text = input("\n>>Prompt: ")
	# 	if input_text == 'exit':
	# 		break
	# 	success = False
	# 	while not success:
	# 		result, success = test_llm(input_text)
	# 		result, success = model(result)
	# 		print(result)
 

	while True:
	# if True:
		input_text = input("\n>>Prompt: ")
		if input_text == 'exit':
			break
		success = False
		while not success:
			result, success = preview(input_text)  
			print("*"*80)
			print(result)
			print("*"*80)
			result, success = model(input_text+"\n"+result)
			if success:
				cmd_processing(result)
			print(result)



if __name__ == '__main__':
    main()
