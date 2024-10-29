from setuptools import find_packages
from setuptools import setup

setup(
    name='dummy_gazebo',
    version='0.0.0',
    packages=find_packages(
        include=('dummy_gazebo', 'dummy_gazebo.*')),
)
