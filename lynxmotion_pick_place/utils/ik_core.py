import math, os, yaml
from .safety import check_joint_limits

def _cfg_path():
    return os.environ.get('ARM_CONFIG_PATH',
        os.path.join(os.path.dirname(__file__), '..', 'config', 'arm_config.yaml'))

def calculate_ik_mm_deg(x, y, z, grip_angle_d=90.0):
    try:
        if z < 10:
            raise ValueError(f"Z {z} below minimum (10mm)")
        cfg_path = _cfg_path()
        if not os.path.exists(cfg_path):
            raise FileNotFoundError(f"Config file {cfg_path} not found")
        with open(cfg_path) as f:
            cfg = yaml.safe_load(f)
        BASE_HGT = cfg['base_height']; HUMERUS = cfg['shoulder_length']
        ULNA = cfg['forearm_length']; GRIPPER = cfg['wrist_length']
        hum_sq, uln_sq = HUMERUS**2, ULNA**2
        grip_r = math.radians(grip_angle_d)
        bas_r = math.atan2(x, y)
        rdist = math.hypot(x, y)
        if rdist < 1e-6: raise ValueError("Target too close")
        y_ik = rdist
        grip_off_z = math.sin(grip_r) * GRIPPER
        grip_off_y = math.cos(grip_r) * GRIPPER
        wrist_z = z - grip_off_z - BASE_HGT
        wrist_y = y_ik - grip_off_y
        s_w_sq = wrist_z**2 + wrist_y**2
        s_w = math.sqrt(max(0.0, s_w_sq))
        max_reach, min_reach = HUMERUS + ULNA, abs(HUMERUS - ULNA)
        if s_w > max_reach or s_w < min_reach: return None
        a1 = math.atan2(wrist_z, wrist_y)
        a2 = math.acos(max(-1, min(1, (hum_sq - uln_sq + s_w_sq)/(2*HUMERUS*s_w))))
        shl_r = a1 + a2
        shl_d = math.degrees(shl_r)
        elb_r = math.acos(max(-1, min(1, (hum_sq + uln_sq - s_w_sq)/(2*HUMERUS*ULNA))))
        elb_d = math.degrees(elb_r)
        elb_dn = -(180.0 - elb_d)
        wri_d = grip_angle_d - elb_dn - shl_d
        angles = {'base': math.degrees(bas_r), 'shoulder': shl_d, 'elbow': elb_dn, 'wrist': wri_d}
        return angles if check_joint_limits(angles) else None
    except Exception:
        return None
