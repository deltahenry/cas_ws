from setuptools import find_packages, setup

package_name = 'motion_control_module'

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
            'motion_control = motion_control_module.motion_control:main',
            'control = motion_control_module.control:main',
            'control_v2 = motion_control_module.control_v2:main',
        ],
    },
)
