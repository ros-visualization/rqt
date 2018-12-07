from setuptools import setup

package_name = 'rqt_gui_py'
setup(
    name=package_name,
    version='0.5.0',
    package_dir={'': 'src'},
    packages=['rqt_gui_py'],
    install_requires=['setuptools'],
    zip_safe=False,
    author='Dirk Thomas',
    author_email='dthomas@osrfoundation.org',
    maintainer='Dirk Thomas',
    maintainer_email='dthomas@osrfoundation.org',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description=(
        'RQT graphic user interface package and supporting files'
    ),
    license='BSD License 2.0',
    tests_require=['pytest'],
)
