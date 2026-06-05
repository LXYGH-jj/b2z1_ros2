from setuptools import setup

package_name = "b2z1_z1_trajectories"

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
    description="Shared Z1 waypoint sequences for official-style and MuJoCo-adjusted demos.",
    license="BSD-3-Clause",
)
