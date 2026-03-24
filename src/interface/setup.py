import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'interface'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        # Install URDF files
        (os.path.join('share', package_name, 'urdf'),
            glob('urdf/*.urdf')),
        # Install config (nav2_params.yaml etc.)
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='omi',
    maintainer_email='omi@todo.todo',
    description='Anibot robot interface: odom, URDF, Nav2 config',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'new_odom =interface.new_odom:main',  #odometery
        ],
    },
)
