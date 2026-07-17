"""simulation.launch.py must only forward arguments the baseline declares.

An IncludeLaunchDescription that passes an argument the included description
does not declare kills the entire launch, and it does so at launch time with a
message about the argument rather than about the mismatch. This repo shipped
that state: the include still named `perception_baseline.launch.py` and still
forwarded `bbox_source`, `monocular_fallback_enabled` and friends long after the
rebuild removed them, so `perception:=true` could not start at all.

Both halves are checked by reading the two files, so this stays honest without
standing up ROS.
"""
import os
import re

from ament_index_python.packages import get_package_share_directory

SIM_LAUNCH = os.path.join(os.path.dirname(__file__), "..", "launch",
                          "simulation.launch.py")


def _baseline_launch_path():
    return os.path.join(
        get_package_share_directory("hyu_perception"),
        "launch", "perception.launch.py")


def _included_file_and_args(source):
    """The filename and launch_arguments of the hyu_perception include."""
    anchor = source.index("get_package_share_directory('hyu_perception')")
    block = source[anchor:]
    end = block.index("launch_arguments=[")
    filename = re.findall(r"'([\w.]+\.launch\.py)'", block[:end])
    args = re.findall(r"\(\s*'([a-z_0-9]+)'\s*,\s*LaunchConfiguration",
                      block[end:block.index("],", end)])
    return filename[0], args


def _declared(source):
    return (set(re.findall(r"DeclareLaunchArgument\(\s*['\"]([a-z_0-9]+)['\"]", source))
            | set(re.findall(r"DeclareLaunchArgument\(\s*name=['\"]([a-z_0-9]+)['\"]",
                             source)))


def test_included_launch_file_exists():
    source = open(SIM_LAUNCH).read()
    filename, _ = _included_file_and_args(source)
    path = os.path.join(os.path.dirname(_baseline_launch_path()), filename)
    assert os.path.isfile(path), (
        f"simulation.launch.py includes {filename}, which is not installed. "
        f"Present: {sorted(os.listdir(os.path.dirname(_baseline_launch_path())))}")


def test_every_forwarded_argument_is_declared():
    source = open(SIM_LAUNCH).read()
    _, forwarded = _included_file_and_args(source)
    declared = _declared(open(_baseline_launch_path()).read())
    unknown = sorted(set(forwarded) - declared)
    assert not unknown, (
        f"simulation.launch.py forwards {unknown} to perception.launch.py, "
        f"which does not declare them. Launch would fail. Declared: "
        f"{sorted(declared)}")
