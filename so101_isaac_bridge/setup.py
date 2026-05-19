from glob import glob
from os.path import join
from setuptools import find_packages, setup

package_name = 'so101_isaac_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (join('share', package_name, 'launch'), glob(join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='HCIS Lab',
    maintainer_email='leochien1110@gmail.com',
    description='Bridge SO-101 MoveIt trajectory actions to Isaac Sim JointState command topics.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'so101_isaac_joint_command_bridge = so101_isaac_bridge.so101_isaac_joint_command_bridge:main'
        ],
    },
)
