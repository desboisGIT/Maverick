from setuptools import find_packages, setup

package_name = "maverick_mapping"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="Maverick",
    maintainer_email="maintainer@example.com",
    description="Voxel map, frontiers and exploration planner, plus an offline harness.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "explore_offline = maverick_mapping.explore_offline:main",
            "mapping_node = maverick_mapping.mapping_node:main",
        ],
    },
)
