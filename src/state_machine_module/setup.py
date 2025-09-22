import os
from setuptools import find_packages, setup

package_name = 'state_machine_module'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), ['launch/fsm.launch.py'])
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
            'multi_threading = state_machine_module.multi_threading:main',
            'fsm_magiccube = state_machine_module.fsm_magiccube:main',
            'fsm_logic = state_machine_module.fsm_logic:main',
            'manual_align = state_machine_module.manual_align:main',
            'run = state_machine_module.run:main',
            'stop = state_machine_module.stop:main',
        ],
    },
)
