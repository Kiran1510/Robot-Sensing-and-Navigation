from setuptools import setup

package_name = 'vn_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/driver.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kiran Sairam',
    maintainer_email='kiransairam1@gmail.com',
    description='ROS2 driver for VectorNav VN-100',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vn_driver = vn_driver.vn_driver:main',
        ],
    },
)
