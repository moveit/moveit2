from setuptools import setup, find_packages

package_name = "moveit_configs_utils"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="MoveIt Release Team",
    maintainer_email="moveit_releasers@googlegroups.com",
    description="Python library for loading moveit config parameters in launch files",
    license="BSD",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
)
