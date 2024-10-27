from setuptools import find_packages
from setuptools import setup

setup(
    name='otto_simulation',
    version='0.0.0',
    packages=find_packages(
        include=('otto_simulation', 'otto_simulation.*')),
)
