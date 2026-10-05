from glob import glob
import os
from setuptools import find_packages, setup

package_name = "tb3_distributed_bringup"
setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Steven Chen",
    maintainer_email="maintainer@example.com",
    description="Bringup for distributed TurtleBot3",
    license="Apache-2.0",
)
