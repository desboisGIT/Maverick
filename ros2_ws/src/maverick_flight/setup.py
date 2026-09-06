from setuptools import find_packages, setup

package_name = "maverick_flight"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Maverick",
    maintainer_email="maintainer@example.com",
    description="Host flight client for Maverick: safety gate, mission client, link-gap policy.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "flight_client = maverick_flight.flight_node:main",
            "inject_goals = maverick_flight.inject_goals:main",
            "px4_monitor = maverick_flight.monitor:main",
        ],
    },
)
