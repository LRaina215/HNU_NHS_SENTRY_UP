'''
Author: Ligcox
Date: 2022-01-27 06:41:01
FilePath: /bubble/src/bubble_core/bubble_protocol/setup.py
LastEditors: Ligcox
LastEditTime: 2022-05-10 19:10:15
License: GNU General Public License v3.0. See LICENSE file in root directory.
Copyright (c) 2022 Birdiebot R&D Department
Shanghai University Of Engineering Science. All Rights Reserved
'''
import os
from glob import glob
from setuptools import setup

package_name = 'bubble_protocol'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
         (os.path.join('share', package_name), glob('launch/*_launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ligcox',
    maintainer_email='ligcox@birdiebot.top',
    description='birdiebot_communication_protocol',
    license='GNU General Public License v3.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "bcp_core = bubble_protocol.bcp_core:main",
        ],
    },
)
