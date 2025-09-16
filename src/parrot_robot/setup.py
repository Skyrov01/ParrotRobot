from setuptools import setup
from setuptools import find_packages
package_name = "parrot_robot"
import os
from glob import glob

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/bringup.launch.py"]),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="YOUR_NAME",
    maintainer_email="you@example.com",
    description="ROS 2 package for Parrot Robot",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "parrot_controller = parrot_robot.parrot_controller:main",
            "parrot_node = parrot_robot.parrot_node:main",
            "head_node = parrot_robot.head_node:main",
            "wings_node = parrot_robot.wings_node:main",
            "sound_node = parrot_robot.sound_node:main",
            "camera_node = parrot_robot.camera_node:main",
            "yolo_node = parrot_robot.yolo_node:main",
            "http_server_node = parrot_robot.http_server_node:main",
        ],
    },
)
