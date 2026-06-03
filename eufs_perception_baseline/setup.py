from glob import glob
from setuptools import setup

package_name = "eufs_perception_baseline"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/docs", glob("docs/*.md")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="IRCV",
    maintainer_email="ircv@todo.invalid",
    description="Perception baseline interface for publishing cone observations to EUFS SLAM.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "perception_baseline_node = eufs_perception_baseline.perception_baseline_node:main",
        ],
    },
)
