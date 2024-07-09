import os
import glob
import yaml

def get_config(config_path=None):
    if config_path is None:
        config_path = './configs/config.yaml'
    assert config_path and os.path.exists(config_path), f'config file does not exist ({config_path})'
    
    with open(config_path, 'r') as f:
        config = yaml.load(f, Loader=yaml.FullLoader)
    config = config['lmp_config']
    return config


def get_path(fname):
    if fname is None:
        return None
    start_dir = os.getcwd()
    search_pattern = os.path.join(start_dir, '**', fname)  # Pattern for recursive search
    found_files = glob.glob(search_pattern, recursive=True)  # Perform the search
    if not found_files:
        print(f'File not found: {fname}')
        return None
    path = found_files[0]
    return path


def load_prompt(prompt_fname):
    full_path = get_path(prompt_fname)
    assert os.path.exists(full_path), f'prompt file does not exist ({prompt_fname})'
    # read file
    with open(full_path, 'r') as f:
        contents = f.read().strip()
    return contents

# test
if __name__ == '__main__':
    get_config()
    print("="*80)
    print(get_path("llama3-8B-instruct-official-fineTuned"))
    print(type(get_path("llama3-8B-instruct-official-fineTuned")))
    print("="*80)
    print(load_prompt("sys_prompt.txt"))    