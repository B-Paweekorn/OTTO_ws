from setuptools import find_packages
from setuptools import setup

setup(
    name='rbt_simulation',
    version='0.0.0',
    packages=find_packages(
        include=('rbt_simulation', 'rbt_simulation.*')),
)