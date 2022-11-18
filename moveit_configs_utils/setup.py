from setuptools import setup
from glob import glob

package_name = "moveit_configs_utils"

setup(
    name=package_name,
    version="2.5.3",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/default_configs", glob("default_configs/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="MoveIt Release Team",
    maintainer_email="moveit_releasers@googlegroups.com",
    description="Python library for loading MoveIt config parameters in launch files",
    license="BSD",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
)
