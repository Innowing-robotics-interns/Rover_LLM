from LMP import LMP
from utils import get_config
import numpy, subprocess, time

import os
import openai
from openai import AzureOpenAI
from dotenv import load_dotenv


def init():
	cfg = get_config('configs/config.yaml')['lmps']
	fixed_vars = {'numpy':numpy, 'subprocess':subprocess, 'time': time} # for third libraries that LLM can access
	variable_vars = {} # for first party libraries (can be other LLM) that a LLM can access
	# allow LMPs to access other LMPs
	# & low-level LLM setup
	lmp_names = [name for name in cfg.keys() if not name in ['coder']] # cfg=lmps_config
	low_level_lmps = {
		k: LMP(k, cfg[k], fixed_vars, variable_vars) #, debug, env_name)
		for k in lmp_names
	}
	variable_vars.update(low_level_lmps)

	# high-level LLM setup
	coder = LMP("coder", cfg['coder'], fixed_vars, variable_vars)
	return coder

def main():
	model = init()
 
	# # objects = ['mouse', 'keyboard']
	# # input_text = test_llm.format_chat_template(f'{objects}\nCommand: put the mouse on the keyboard.')
	# # input_text = f'what is the sum of 2 and 3?'exec
	# # input_text = f'can you go to the 3D printer station and get the key for Event Hall, then go to the Event Hall and unlock the door?'
	# # input_text = f'arm move to (1.2,1.0,0.7), then go left by a small distance, finally rotate in pitch-axis by a small angle.'
	# # input_text = f'draw me an arc with radius 3.0, then move backward.'
	# while True:
	# # if True:
	# 	input_text = input("\n>>Prompt: ")
	# 	if input_text == 'exit':
	# 		break
	# 	success = False
	# 	while not success:
	# 		result, success=model(input_text)
	# 		print(result)


	# test
	test_llm = LMP("test", get_config('configs/config.yaml')['lmps']['test'])
	while True:
		# if True:
		input_text = input("\n>>Prompt: ")
		if input_text == 'exit':
			break
		success = False
		while not success:
			result, success = test_llm(input_text)
			result, success = model(result)
			print(result)
 


if __name__ == '__main__':
    main()
