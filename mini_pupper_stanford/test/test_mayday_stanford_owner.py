from pathlib import Path


OWNER = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "mayday_stanford_owner"
)


def source():
    return OWNER.read_text(encoding="utf-8")


def restore_source():
    text = source()
    start = text.index("restore()")
    end = text.index("handle_stop()", start)

    return text[start:end]


def test_owner_has_no_robot_service_dependency():
    text = source()

    for forbidden in (
        "systemctl is-active --quiet robot.service",
        "systemctl stop robot.service",
        "systemctl start robot.service",
        "journalctl -u robot.service",
        "Waiting for L1 to activate robot.",
        "robot.service",
    ):
        assert forbidden not in text


def test_acquisition_keeps_exact_normal_writer_checks():
    text = source()

    assert "find_exactly_one" in text
    assert '"$CHAMP_PATTERN"' in text
    assert '"$SERVO_PATTERN"' in text
    assert "require_unfrozen" in text
    assert text.count('"CHAMP"') >= 2
    assert text.count('"servo_interface"') >= 2


def test_acquisition_keeps_adapter_duplicate_rejection_and_zero_before_freeze():
    text = source()

    adapter_rejection = text.index(
        'fail "Stanford ROS adapter already running."'
    )
    zero = text.index("bridge_stop", adapter_rejection)
    freeze_champ = text.index('kill -STOP "$CHAMP_PID"')
    freeze_servo = text.index('kill -STOP "$SERVO_PID"')

    assert zero < freeze_champ < freeze_servo


def test_acquisition_verifies_freeze_before_adapter_start_and_ready():
    text = source()

    frozen_champ = text.index("require_frozen", text.index('kill -STOP "$SERVO_PID"'))
    frozen_servo = text.index('"servo_interface"', frozen_champ)
    gate = text.index("exclusive Stanford ownership gate established")
    adapter_start = text.index('"$ADAPTER"', gate)
    subscription = text.index("ros2 topic info", adapter_start)
    ready = text.index("systemd-notify", subscription)

    assert frozen_champ < frozen_servo < gate < adapter_start
    assert subscription < ready
    assert 'SUBS:-0' in text


def test_restore_preserves_zero_adapter_stop_and_normal_writer_order():
    text = restore_source()

    first_zero = text.index("bridge_stop")
    adapter_stop = text.index('kill -INT "$ADAPTER_PID"')
    second_zero = text.index("bridge_stop", first_zero + 1)
    champ_identity = text.index('"/proc/$CHAMP_PID/cmdline"')
    champ_resume = text.index('kill -CONT "$CHAMP_PID"')
    settle = text.index("sleep 1.25")
    servo_identity = text.index('"/proc/$SERVO_PID/cmdline"')
    servo_resume = text.index('kill -CONT "$SERVO_PID"')

    assert first_zero < adapter_stop < second_zero
    assert champ_identity < champ_resume < settle
    assert settle < servo_identity < servo_resume


def test_cleanup_and_systemd_notify_guards_remain():
    text = source()

    assert "trap restore EXIT" in text
    assert "trap handle_stop INT TERM HUP" in text
    assert "MAYDAY_STANFORD_HARDWARE" in text
    assert "command_timeout" in text
    assert "max_linear_x" in text
    assert "max_angular_z" in text
    assert text.index("exclusive Stanford ownership gate established") < text.index(
        "systemd-notify"
    )
