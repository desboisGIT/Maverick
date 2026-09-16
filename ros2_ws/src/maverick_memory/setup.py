from setuptools import find_packages, setup

package_name = "maverick_memory"

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
    description="Scene graph: detection lifting, object store, room segmentation and typing.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "eval_offline = maverick_memory.eval_offline:main",
            "memory_node = maverick_memory.memory_node:main",
        ],
    },
)
