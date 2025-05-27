from setuptools import find_packages, setup

package_name = 'joint_trajectory_to_position_cmd'

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
            'joint_trajectory_to_position_cmd_node = joint_trajectory_to_position_cmd.joint_trajectory_to_position_cmd_node:main',
        ],
    },
)
