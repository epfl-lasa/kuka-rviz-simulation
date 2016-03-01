#!/usr/bin/eny python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['iai_control_utils'],
    package_dir={'': 'src'}
)

setup(**d)