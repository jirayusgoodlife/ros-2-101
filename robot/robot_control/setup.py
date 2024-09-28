from setuptools import setup

package_name = 'robot_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'pyzmq',  # Add pyzmq as a dependency to ensure it is installed
    ],
    zip_safe=True,
    maintainer='naphon',
    maintainer_email='naphon.851@gmail.com',
    description='ROS 2 package for controlling a robot using keyboard commands',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'ros2_controller = robot_control.ros2_controller:main'
        ],
    },
)
