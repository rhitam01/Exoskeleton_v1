from setuptools import setup, find_packages

package_name = 'nmotion_transport'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(include=[package_name, f'{package_name}.*']),  # includes core
    package_dir={package_name: package_name},
    package_data={
        # Include all .so files inside core/lib
        'nmotion_transport.core': ['lib/*.so', 'lib/oss_licenses.txt'],
    },
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='NMotion',
    maintainer_email='amarnath@nmotion.in',
    description='Nmotion transport library for ROS2',
    license='Apache-2.0',
)
