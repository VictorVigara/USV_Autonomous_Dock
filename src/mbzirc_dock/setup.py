import os
from glob import glob
from setuptools import setup

package_name = 'mbzirc_dock'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}', glob('launch/*launch.[pxy][yma]*')),
        (f'share/{package_name}/config', glob('config/**')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='developer',
    maintainer_email='wojcikmichal98@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dock_server = mbzirc_dock.main:main'
        ],
    },
)
