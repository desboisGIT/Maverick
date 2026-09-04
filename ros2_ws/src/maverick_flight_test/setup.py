from setuptools import find_packages, setup

package_name = "maverick_flight_test"

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
    description="S0 flight test: offboard take off, hover and land against PX4 SITL.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "hover_test = maverick_flight_test.hover_test:main",
        ],
    },
)
