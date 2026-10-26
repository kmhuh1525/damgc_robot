from glob import glob
from setuptools import find_packages, setup


package_name = "follower_supply_perception"


setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="kde",
    maintainer_email="kde@example.com",
    description=(
        "AprilTag-relative perception and alignment-state support for the follower robot."
    ),
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
       "console_scripts": [
         "camera_pair_node = follower_supply_perception.camera_pair_node:main",
   	 "apriltag_approach_node = "
   	 "follower_supply_perception.apriltag_approach_node:main",
	],
    },
)
