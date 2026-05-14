from setuptools import setup
from glob import glob
import os

package_name = 'washbot_decision'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools', 'pyyaml'],
    zip_safe=True,
    maintainer='yuandong',
    maintainer_email='yuandong@example.com',
    description='WashBot decision layer for interaction point scan and rectangle patrol.',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'rect_patrol_node = washbot_decision.rect_patrol_node:main',
        'center_scan_odom_node = washbot_decision.center_scan_odom_node:main',
    ],
},
)
