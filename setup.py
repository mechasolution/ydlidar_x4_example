import glob
import os

from setuptools import setup

package_name = "ydlidar_x4_example"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob.glob("launch/*.launch.py"),
        ),
        (os.path.join("share", package_name, "param"), glob.glob("param/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Max Cha",
    maintainer_email="max@mechasolution.com",
    description="YDLiDAR X4 example for ROS 2",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["example_node = ydlidar_x4_example.example_node:main"],
    },
)
