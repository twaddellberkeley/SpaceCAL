from setuptools import setup

package_name = 'projector_exec'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
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
            'proj_node_0 = projector_exec.proj_node_0:main',
            'proj_node_1 = projector_exec.proj_node_1:main',
            'proj_node_2 = projector_exec.proj_node_2:main',
            'proj_node_3 = projector_exec.proj_node_3:main',
            'proj_node_4 = projector_exec.proj_node_4:main'
        ],
    },
)
