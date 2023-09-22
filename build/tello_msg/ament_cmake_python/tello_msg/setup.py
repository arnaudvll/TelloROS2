from setuptools import find_packages
from setuptools import setup

setup(
    name='tello_msg',
    version='0.0.1',
    packages=find_packages(
        include=('tello_msg', 'tello_msg.*')),
)
