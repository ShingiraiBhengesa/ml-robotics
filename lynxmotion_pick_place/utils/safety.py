import os, yaml
def _cfg_path():
    return os.environ.get('ARM_CONFIG_PATH',
        os.path.join(os.path.dirname(__file__), '..', 'config', 'arm_config.yaml'))

def load_limits():
    with open(_cfg_path(), 'r') as f:
        return yaml.safe_load(f)['limits']

def check_joint_limits(angles_deg: dict) -> bool:
    lim = load_limits()
    for j, val in angles_deg.items():
        if j not in lim:
            continue
        mn, mx = float(lim[j]['min']), float(lim[j]['max'])
        if val < mn or val > mx:
            return False
    return True
