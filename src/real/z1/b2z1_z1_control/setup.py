from setuptools import setup

package_name = "b2z1_z1_control"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="liu",
    maintainer_email="liu@example.com",
    description="Z1 real-robot control package aligned with official z1_ros2 waypoint execution.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "z1_waypoint_test = b2z1_z1_control.z1_waypoint_test:main",
        ],
    },
)
