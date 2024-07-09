from LMP import LMP
from utils import get_config
import numpy as np


def main():
    cfg = get_config('configs/config.yaml')['lmps']
    fixed_vars = {'np':np}
    test_llm = LMP("test", cfg['coder'])
    test_llm.load_model()
    objects = ['mouse', 'keyboard']
    # input_text = test_llm.format_chat_template(f'{objects}\nCommand: put the mouse on the keyboard.')
    input_text = test_llm.format_chat_template(f'Query: what is the sum of 2 and 3?')
    success = False
    while not success:
        result, success=test_llm.generate(input_text)
        print(result)
    
    

if __name__ == '__main__':
    main()
