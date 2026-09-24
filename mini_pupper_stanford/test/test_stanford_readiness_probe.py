from mini_pupper_stanford.stanford_readiness_probe import FAIL, READY, WAIT, evaluate


def test_readiness_decisions_are_exact_and_pid_bound():
    assert evaluate([], [], None, 1) == WAIT
    assert evaluate(["/stanford_cmd_vel"], [], None, 1) == WAIT
    subs = [("/cmd_vel", ["geometry_msgs/msg/Twist"])]
    assert evaluate(["/stanford_cmd_vel"], subs, None, 1) == WAIT
    assert evaluate(["/stanford_cmd_vel"], subs, "1", 1) == FAIL
    assert evaluate(["/stanford_cmd_vel"], subs, 2, 1) == FAIL
    assert evaluate(["/stanford_cmd_vel"], subs, 1, 1) == READY
    assert evaluate(["/stanford_cmd_vel", "/stanford_cmd_vel"], subs, 1, 1) == FAIL
