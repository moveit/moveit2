from setuptools import setup

package_name = 'moveit_commander'

setup(
    name=package_name,
    version='1.2.0',
    packages=[package_name],
    package_dir={package_name: "src/"+package_name},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'transforms3d'],
    zip_safe=True,
    maintainer='Larry Lu',
    maintainer_email='larrylu0426@gmail.com',
    description='Python interfaces to MoveIt',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
