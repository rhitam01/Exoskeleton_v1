from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'cubemars_v1'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/launch', ['launch/hip_system.launch.py']),(
            os.path.join('share', package_name, 'data'),
            glob('data/*')
        ),
],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rudhratej',
    maintainer_email='rudhratejsingh6@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        'hip_const_pub=cubemars_v1.hip_constant_pub:main',
        'hip_cont_pub=cubemars_v1.hip_continous_pub:main',
        'hip_logger_sub=cubemars_v1.hip_logger_sub:main',
        'encoder_pub=cubemars_v1.encoder_hip_pub:main',
        ],
    },
)
