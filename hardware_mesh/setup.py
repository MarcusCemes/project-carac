from glob import glob
from os import path
from setuptools import find_packages, setup

package_name = "hardware_mesh"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (path.join("share", package_name, "launch"), glob("launch/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Marcus Cemes",
    maintainer_email="marcus.cemes@epfl.ch",
    description="TODO: Package description",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "wind_shape = hardware_mesh.nodes.wind_shape:main",
            "robot_arm = hardware_mesh.nodes.robot_arm:main",
        ],
    },
)
