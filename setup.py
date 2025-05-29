from setuptools import find_packages, setup

package_name = 'robot_control_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lhj',
    maintainer_email='hojun7889@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'relay_node = robot_control_bridge.relay_node:main',
            'inference = robot_control_bridge.inference:main',
            'payload_publisher = robot_control_bridge.payload_publisher:main',
            'sim_test = robot_control_bridge.sim_test:main',
        ],
    },
)
