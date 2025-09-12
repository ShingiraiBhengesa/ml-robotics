from lynxmotion_pick_place.utils.safety import check_joint_limits
def test_safety_ok():    assert check_joint_limits({'base':0,'shoulder':45,'elbow':0,'wrist':0})
def test_safety_fail():  assert not check_joint_limits({'base':200,'shoulder':45,'elbow':0,'wrist':0})
