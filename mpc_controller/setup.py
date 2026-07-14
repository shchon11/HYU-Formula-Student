from setuptools import setup

package_name = "mpc_controller"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config",
         ["config/mpc_controller.yaml", "config/mpc_controller_skidpad.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="HYU Formula Student",
    maintainer_email="shchon724@gmail.com",
    description="LTV-MPC drive controller (drop-in replacement for pure pursuit).",
    license="MIT",
    entry_points={
        "console_scripts": [
            "mpc_controller_node = mpc_controller.mpc_controller_node:main",
        ],
    },
)
