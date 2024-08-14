import os
import glob
import yaml
from scipy.spatial.transform import Rotation as R

##################################################################################
# system functions
# Find file path from file name
# fname: file name to search
# return: file path if found, None otherwise
def get_path(fname=None):
    if not fname:
        return None
    
    start_dir = os.getcwd()
    search_pattern = os.path.join(start_dir, '**', fname)  # Pattern for recursive search
    found_files = glob.glob(search_pattern, recursive=True)  # Perform the search
    if not found_files:
        print(f'File not found: {fname}')
        return None
    path = found_files[0]
    return path


# Load config file (.ymal file) from file path / name
# config_path: path to config file
# config_name: name of config file
# return: config dictionary
def get_config(config_name=None, config_path='./configs/config.yaml'):
    assert config_name and os.path.exists(config_path), f'config file does not exist ({config_name, config_path})'
    
    if config_name:
        config_path = get_path(config_name)
    with open(config_path, 'r') as f:
        config = yaml.load(f, Loader=yaml.FullLoader)
    config = config['lmp_config']
    return config


# Load prompt file from file name
# prompt_fname: name of prompt file
# prompt_fpath: path to prompt file
# return: content of prompt file
def load_prompt(prompt_fname:str, prompt_fpath=None)->str:
    if not prompt_fpath:
        prompt_fpath = get_path(prompt_fname)
    assert prompt_fpath, f'prompt file does not exist ({prompt_fname})'

    with open(prompt_fpath, 'r') as f:
        contents = f.read().strip()
    return contents
##################################################################################


##################################################################################
# Math functions
# Convert quaternion to euler angles
def to_euler(quaternion):
    r = R.from_quat(quaternion)
    euler = r.as_euler('xyz', degrees=True)
    return euler

# Convert euler angles to quaternion
def to_quaternion(euler):
    r = R.from_euler('xyz', euler, degrees=True)
    quaternion = r.as_quat()
    return list(quaternion)

##################################################################################


# test
if __name__ == '__main__':
    cfg = get_config(config_name="config.yaml")
    print("="*80)
    print(get_path("llama3-8B-instruct-official-fineTuned"))
    print(type(get_path("llama3-8B-instruct-official-fineTuned")))
    print("="*80)
    print(load_prompt("coder_sys_prompt.txt"))    