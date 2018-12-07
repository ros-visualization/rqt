from setuptools import setup

package_name = 'rqt'
setup(
    name=package_name,
    version='0.5.0',
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
)
