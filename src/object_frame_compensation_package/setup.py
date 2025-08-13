from setuptools import setup, find_packages

package_name = 'object_frame_compensation_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/object_frame_compensation.launch.py']),
        ('share/' + package_name + '/config', ['config/object_frame_params.yaml']),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='ROS2 Object Frame Compensation node using object coordinate system method for robotic arm positioning',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'object_frame_compensation_node = object_frame_compensation_package.object_frame_compensation_node:main'
        ],
    },
)