from setuptools import find_packages, setup

package_name = 'task_module'

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
            'init = task_module.init:main',
            'component_control = task_module.component_control:main',
            'rough_align = task_module.rough_align:main',
            'precise_align = task_module.precise_align:main',
            'pick = task_module.pick:main',
            'assembly = task_module.assembly:main',
            'recipe_node = task_module.recipe_node:main',
            'compensate_v2 = task_module.compensate_v2:main',
            'compensate_origin = task_module.compensate_origin:main',
        ],
    },
)
