from lynxmotion_pick_place.utils.ik_core import calculate_ik_mm_deg
def test_ik_reachable():
    ang = calculate_ik_mm_deg(0.0, 150.0, 100.0, 90.0)
    assert ang is not None and all(k in ang for k in ['base','shoulder','elbow','wrist'])
def test_ik_unreachable():
    assert calculate_ik_mm_deg(0.0, 500.0, 0.0, 90.0) is None
