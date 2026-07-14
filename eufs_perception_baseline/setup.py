from setuptools import setup

package_name = "eufs_perception_baseline"

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
            ["config/perception_baseline.yaml"],
        ),
        (
            f"share/{package_name}/launch",
            ["launch/perception_baseline.launch.py"],
        ),
        (
            f"share/{package_name}/docs",
            [
                "docs/iit_bombay_baseline_design.md",
                "docs/perception_baseline_usage.md",
            ],
        ),
    ],
    install_requires=[
        "setuptools",
        "PyYAML>=5.0",
        "ultralytics==8.4.60",
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
                "perception_baseline_node = "
                "eufs_perception_baseline.perception_baseline_node:main"
            ),
            (
                "yolov8_bbox_node = "
                "eufs_perception_baseline.yolov8_bbox_node:main"
            ),
        ],
    },
)
