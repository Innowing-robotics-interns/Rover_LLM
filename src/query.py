# next formatting update: 23/08/2024

from LMP import LMP
from utils import get_config
from BaseMotion import go_Xaxis, go_Yaxis, go_to_point, circle, BASEMOTION
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

    # init
	rclpy.init()
	preview, model = model_init() 
	MoblieBase = BASEMOTION()
	# history stored in format of [input_text, result]
	# max length of query history is 10
	query_history = {0: ["", ""]}
	query_history_max_len = 3
	query_history_idx = 0

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
			print(input_text+"\n"+result)
			print("*"*80)
			model_input = f'Last operation:\nQuery:{query_history[query_history_idx][0]}\nResult:{query_history[query_history_idx][1]}\n\nCurrent operation: {input_text}\n{result}'
			result, success = model(model_input)
			if success:
				query_history_idx = (query_history_idx + 1) % query_history_max_len
				query_history[query_history_idx] = [input_text, result]
				MoblieBase.cmd_processing(result)
			print(result)



if __name__ == '__main__':
    main()
