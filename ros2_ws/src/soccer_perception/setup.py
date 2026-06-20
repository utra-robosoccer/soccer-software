from setuptools import find_packages, setup

package_name = "soccer_perception"

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
    description="MiniBot perception: detector + field-line segmentation + 3D projection.",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "detector_node = soccer_perception.detector_node:main",
            "fieldline_node = soccer_perception.fieldline_node:main",
            "projection_node = soccer_perception.projection_node:main",
        ],
    },
)
