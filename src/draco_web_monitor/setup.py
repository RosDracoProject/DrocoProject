from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'draco_web_monitor'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 템플릿 파일들 포함
        (os.path.join('lib', package_name, 'templates'), 
            glob('draco_web_monitor/templates/*')),
    ],
    install_requires=['setuptools', 'flask', 'psutil', 'netifaces'],
    zip_safe=True,
    maintainer='youngmo',
    maintainer_email='youngmo123@hanmail.net',
    description='Web monitoring system for Draco pointcloud compression performance and bandwidth measurement',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'web_monitor_node = draco_web_monitor.web_monitor_node:main',
        ],
    },
)
