#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=[
        'koco_conversions',
        'koco_geometry',
        'koco_action',
        'koco_proxies',
        'koco_toolbox',
    ],
    package_dir={'': 'src'}
)

setup(**setup_args)
