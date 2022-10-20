from setuptools import setup

package_name = 'level_motor_controller'
submodules = 'level_motor_controller/submodules'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, submodules],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cjc',
    maintainer_email='christianjc_09@yahoo.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'level_motor_node = level_motor_controller.level_motor_node:main'
        ],
    },
)
