from setuptools import find_packages, setup
import os
from glob import glob
from setuptools import setup

package_name = 'dev_drone'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
         # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='arnaud.ville',
    maintainer_email='arnaud.ville@cpe.fr',
    description="Controle d'un drone quadrirotor",
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control = dev_drone.control:main',
            'tello_behavior = dev_drone.tello_behavior:main'
        ],
    },
)
