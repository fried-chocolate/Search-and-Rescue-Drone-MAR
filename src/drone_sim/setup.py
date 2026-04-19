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
        # Launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        # World SDF
        (os.path.join('share', package_name, 'worlds'),
            glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'worlds', 'textures'),
            glob('worlds/textures/*')),
        # Root models (drone.sdf)
        (os.path.join('share', package_name, 'models'),
            glob('models/*.sdf')),
        # ── Quadrotor mesh model ──────────────────────────────────────────
        (os.path.join('share', package_name, 'models', 'quadrotor'),
            glob('models/quadrotor/model.*')),
        (os.path.join('share', package_name, 'models', 'quadrotor', 'meshes'),
            glob('models/quadrotor/meshes/*')),
        # ── Hatchback Red mesh model ──────────────────────────────────────
        (os.path.join('share', package_name, 'models', 'hatchback_red'),
            glob('models/hatchback_red/model.*')),
        (os.path.join('share', package_name, 'models', 'hatchback_red', 'meshes'),
            glob('models/hatchback_red/meshes/*')),
        (os.path.join('share', package_name, 'models', 'hatchback_red',
                      'materials', 'textures'),
            glob('models/hatchback_red/materials/textures/*')),
        # ── Standing Person mesh model ────────────────────────────────────
        (os.path.join('share', package_name, 'models', 'person_standing'),
            glob('models/person_standing/model.*')),
        (os.path.join('share', package_name, 'models', 'person_standing', 'meshes'),
            glob('models/person_standing/meshes/*')),
        (os.path.join('share', package_name, 'models', 'person_standing',
                      'materials', 'textures'),
            glob('models/person_standing/materials/textures/*')),
        # ── SAR drone model package (for world include at startup) ───────
        (os.path.join('share', package_name, 'models', 'sar_drone'),
            glob('models/sar_drone/model.*')),
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
