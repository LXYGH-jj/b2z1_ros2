from setuptools import setup

package_name = "b2z1_examples"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="liu",
    maintainer_email="liu@example.com",
    description="Minimal MuJoCo validation entrypoints for B2 and Z1 command semantics.",
    license="BSD-3-Clause",
    entry_points={
        "console_scripts": [
            "b2_stand_validation = b2z1_examples.b2_stand_validation:main",
            "z1_waypoint_validation = b2z1_examples.z1_waypoint_validation:main",
        ],
    },
)
