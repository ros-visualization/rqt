#!/usr/bin/env python3

from distutils.core import setup

from catkin_pkg.python_setup import generate_distutils_setup

package_name = "rqt_gui"

d = generate_distutils_setup(
    packages=[package_name],
    package_dir={'': 'src'},
    scripts=['bin/rqt'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
)

setup(**d)
