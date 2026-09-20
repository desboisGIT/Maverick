import os
from glob import glob

from setuptools import find_packages, setup

package_name = "maverick_vo"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Maverick",
    maintainer_email="maintainer@example.com",
    description="Host-side RGB-D visual odometry (rtabmap) feeding PX4 as external vision.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "camera_unpack = maverick_vo.camera_unpack:main",
            "vo_adapter = maverick_vo.adapter_node:main",
        ],
    },
)
