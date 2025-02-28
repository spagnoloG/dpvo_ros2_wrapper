from setuptools import setup, find_packages
import os
from glob import glob

package_name = "dpvo_ros2_wrapper"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*.py")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    author="Gasper Spagnolo",
    author_email="spagnolo.gasper@gmail.com",
    maintainer="Gasper Spagnolo",
    maintainer_email="spagnolo.gasper@gmail.com",
    description="A ROS2 wrapper for DPVO (Deep Patch Visual Odometry/SLAM).",
    license="GPLv3",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "dpvo_ros2_publisher = dpvo_ros2_wrapper.dpvo_ros2_publisher:main"
        ],
    },
)
