from setuptools import find_packages, setup

package_name = 'io_module'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='henry',
    maintainer_email='HENRY.HY.CHANG@deltaww.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'depth_rangefinder = io_module.depth_rangefinder:main',
            'height = io_module.height:main',
            'origin_fork = io_module.origin_fork:main',
            'io_node = io_module.io_node:main',
        ],
    },
)
