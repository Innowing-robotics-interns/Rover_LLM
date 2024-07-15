from LMP import LMP
from utils import get_config
import numpy as np


def main():
    cfg = get_config('configs/config.yaml')['lmps']
    
    
    
    
    fixed_vars = {'np':np} # for third libraries that LLM can access
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
    coder = LMP("test", cfg['coder'], fixed_vars, variable_vars)
    
    
    # objects = ['mouse', 'keyboard']
    # input_text = test_llm.format_chat_template(f'{objects}\nCommand: put the mouse on the keyboard.')
    # input_text = f'what is the sum of 2 and 3?'
    # input_text = f'can you go to the 3D printer station and get the key for Event Hall, then go to the Event Hall and unlock the door?'
    # input_text = f'arm move to (1.2,1.0,0.7), then go left by a small distance, finally rotate in pitch-axis by a small angle.'
    input_text = f'arm move to (1.2,1.0,0.7), then move to right for a bit, finally rotate a little in roll-axis.'
    while True:
      input_text = input(">>Prompt:")
      success = False
      while not success:
          result, success=coder(input_text)
          print(result)
    
    

if __name__ == '__main__':
    main()
