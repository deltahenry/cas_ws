import os
from setuptools import find_packages, setup

package_name = 'ui_module'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), ['launch/ui.launch.py'])
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
            'magic_cube = ui_module.ui_magic_cube:main',
            "ui_motor_control = ui_module.ui_motor_control:main",
            "ui_fsm_logic = ui_module.ui_fsm_logic:main",
            "ui_forklift = ui_module.ui_forklift:main",
            "ui_recipe = ui_module.ui_recipe:main",
            "ui_comfirm = ui_module.ui_comfirm:main",
            "ui_limit = ui_module.ui_limit:main",
        ],
    },
)
