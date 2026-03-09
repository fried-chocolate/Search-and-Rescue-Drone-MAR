import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'drone_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'worlds'),
            glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'worlds', 'textures'),
            glob('worlds/textures/*')),
        (os.path.join('share', package_name, 'models'),
            glob('models/*.sdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='poojareddyv-pes2ug23cs413',
    maintainer_email='poojareddyv-pes2ug23cs413@todo.todo',
    description='Search-and-rescue drone simulation',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'takeoff = drone_sim.takeoff:main',
            'camera_viewer = drone_sim.camera_viewer:main',
        ],
    },
)
