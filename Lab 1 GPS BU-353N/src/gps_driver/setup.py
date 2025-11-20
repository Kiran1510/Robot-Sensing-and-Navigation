from setuptools import find_packages, setup

package_name = 'gps_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/gps_launch.py']),
    ],
    install_requires=['setuptools', 'serial', 'utm'],
    zip_safe=True,
    maintainer='kiran',
    maintainer_email='kiran@todo.todo',
    description='ROS2 Driver for GPS puck',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'driver = gps_driver.driver:main',
        ],
    },
)

