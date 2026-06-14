from setuptools import find_packages, setup

package_name = "soccer_localization"

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
    description="Two-tier localization: Tier-1 EKF odometry + Tier-2 MCL field pose.",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "ekf_node = soccer_localization.ekf_node:main",
            "mcl_node = soccer_localization.mcl_node:main",
        ],
    },
)
