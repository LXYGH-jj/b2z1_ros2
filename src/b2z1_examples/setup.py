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
    description="Minimal examples for coordinated B2 and Z1 commands.",
    license="BSD-3-Clause",
    entry_points={
        "console_scripts": [
            "b2_only_stand = b2z1_examples.b2_only_stand:main",
            "b2_only_stand_then_down = b2z1_examples.b2_only_stand_then_down:main",
            "z1_only_reach = b2z1_examples.z1_only_reach:main",
        ],
    },
)
