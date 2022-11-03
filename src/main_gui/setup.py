from setuptools import setup

package_name = 'main_gui'
submodules = 'main_gui/submodules'
Components = 'main_gui/submodules/Components'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, submodules, Components],
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
            'main_gui_node = main_gui.main_gui_node:main'
        ],
    },
)
