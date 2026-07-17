from setuptools import setup

package_name = "hyu_perception"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [f"resource/{package_name}"],
        ),
        (
            f"share/{package_name}",
            ["package.xml", "THIRD_PARTY_NOTICES.md"],
        ),
        (
            f"share/{package_name}/config",
            ["config/perception.yaml"],
        ),
        (
            f"share/{package_name}/launch",
            ["launch/perception.launch.py"],
        ),
        (
            f"share/{package_name}/docs",
            [
                "docs/INTEGRATION.md",
                "docs/iit_bombay_baseline_design.md",
                "docs/perception_baseline_usage.md",
            ],
        ),
        # The cone pose weight ships with the package so the node has no
        # dependency on a path outside the workspace.
        (
            f"share/{package_name}/models/cone_pose_8kpt/weights",
            ["models/cone_pose_8kpt/weights/best.pt"],
        ),
        (
            f"share/{package_name}/scripts",
            [
                "scripts/measure_sim_rates.py",
                "scripts/evaluate_perception_tiers.py",
                "scripts/fit_mono_depth_curve.py",
            ],
        ),
    ],
    scripts=[
        "scripts/measure_sim_rates.py",
        "scripts/evaluate_perception_tiers.py",
        "scripts/fit_mono_depth_curve.py",
    ],
    install_requires=[
        "setuptools",
        "PyYAML>=5.0",
        # Floor is the oldest build verified to load the cone-pose checkpoint
        # here (8.4.17 runs it and publishes keypoints). The >=8.4.90 pin this
        # arrived with was never executed against ROS and only blocked startup.
        "ultralytics>=8.4.17,<9",
    ],
    zip_safe=True,
    maintainer="IRCV",
    maintainer_email="ircv@todo.invalid",
    description=(
        "Perception baseline interface for publishing cone observations "
        "to EUFS SLAM."
    ),
    license="MIT",
    entry_points={
        "console_scripts": [
            (
                "perception_node = "
                "hyu_perception.perception_node:main"
            ),
            (
                "yolov8_bbox_node = "
                "hyu_perception.yolov8_bbox_node:main"
            ),
        ],
    },
)
