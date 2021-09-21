from setuptools import setup

package_name = 'JNode_MotorController'

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
    description='Runs motors with given address parameter',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motorRun = JNode_MotorController.motorController:main'
        ],
    },
)
