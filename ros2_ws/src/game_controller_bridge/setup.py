from setuptools import find_packages, setup

package_name = "game_controller_bridge"

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
    maintainer="RoboCup Humanoid Team",
    maintainer_email="team@robocup.example",
    description="RoboCup GameController UDP bridge (ports 3838/3939).",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "gc_bridge_node = game_controller_bridge.gc_bridge_node:main",
        ],
    },
)
