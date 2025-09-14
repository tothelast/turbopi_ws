import yaml
import os

# Get the directory where this script is located
script_dir = os.path.dirname(os.path.abspath(__file__))
lab_file_path = os.path.join(script_dir, 'lab_config.yaml')
servo_file_path = os.path.join(script_dir, 'servo_config.yaml')

def get_yaml_data(yaml_file):
    file = open(yaml_file, 'r', encoding='utf-8')
    file_data = file.read()
    file.close()
    
    data = yaml.load(file_data, Loader=yaml.FullLoader)
    
    return data

def save_yaml_data(data, yaml_file):
    file = open(yaml_file, 'w', encoding='utf-8')
    yaml.dump(data, file)
    file.close()
