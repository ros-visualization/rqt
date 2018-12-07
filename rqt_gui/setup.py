from setuptools import setup

package_name = 'rqt_gui'
setup(
    name=package_name,
    version='0.5.0',
    package_dir={'': 'src'},
    packages=['rqt_gui'],
    data_files=[
        ('share/' + package_name + '/resource', [
            'resource/rqt.graffle',
            'resource/rqt.png',
            'resource/rqt.svg',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
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
    #scripts=['bin/rqt', 'bin/rqt_gui'],
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rqt_gui = rqt_gui.main:main',
            'rqt = rqt_gui.main:main',
        ],
    },
)
