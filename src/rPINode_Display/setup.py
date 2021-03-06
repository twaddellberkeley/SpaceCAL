from setuptools import setup

package_name = 'rPINode_Display'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='spacecal',
    maintainer_email='twaddell@berkeley.edu',
    description='Node package that will display to given display',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'videoDisplay = rPINode_Display.rPI_Display:main'
        ],
    },
)
