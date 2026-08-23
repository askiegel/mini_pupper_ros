import re
from pathlib import Path


PARAM = (
    Path(__file__).resolve().parents[1]
    / "param"
    / "mayday_guarded_navigation.yaml"
)


def _top_level_section(
    text,
    name,
):
    pattern = re.compile(
        rf"(?ms)^{re.escape(name)}:\n"
        rf"(.*?)"
        rf"(?=^[A-Za-z_][A-Za-z0-9_]*:\n|\Z)"
    )

    match = pattern.search(text)

    assert match is not None

    return match.group(1)


def _transform_tolerance(section):
    values = re.findall(
        r"(?m)^\s+transform_tolerance:"
        r"\s*([0-9.]+)\s*$",
        section,
    )

    assert len(values) == 1

    return float(values[0])


def test_guarded_costmaps_allow_measured_tf_delay():
    text = PARAM.read_text()

    local = _top_level_section(
        text,
        "local_costmap",
    )

    global_costmap = _top_level_section(
        text,
        "global_costmap",
    )

    assert (
        _transform_tolerance(local)
        == 1.50
    )

    assert (
        _transform_tolerance(global_costmap)
        == 1.50
    )
