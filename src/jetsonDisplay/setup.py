from setuptools import setup
import os
from glob import glob

package_name = 'jetsonDisplay'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
	('lib/python3.8/site-packages/' + package_name,['spaceCalMW.ui']), # Replace [file] with correct file
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='spacecal',
    maintainer_email='twaddell@berkeley.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'guiMain = jetsonDisplay.spaceCalApp_v2_ros:main'   # Replace "spaceCalApp_v2_ros" with correct python file
        ],
    },
)
