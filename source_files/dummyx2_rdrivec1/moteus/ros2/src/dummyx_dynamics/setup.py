from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'dummyx_dynamics'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Install URDF files
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='DummyX2 Team',
    maintainer_email='user@example.com',
    description='Pinocchio-based dynamics controller for DummyX2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gravity_controller = dummyx_dynamics.gravity_controller_node:main',
        ],
    },
)
