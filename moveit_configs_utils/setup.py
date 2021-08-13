from setuptools import setup

package_name = "moveit_configs_utils"

setup(
    name=package_name,
    version="0.0.0",
    package_dir={"": "src"},
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    author="TODO",
    author_email="TODO",
    maintainer="TODO",
    maintainer_email="TODO",
    keywords=["ROS2"],
    classifiers=[
        "Intended Audience :: Developers",
        "License :: OSI Approved :: BSD",
        "Programming Language :: Python",
        "Topic :: Software Development",
    ],
    description="TODO",
    license="BSD",
)
