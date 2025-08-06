from setuptools import find_packages, setup
import os
from glob import glob


package_name = 'vision_module'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('lib', package_name, 'template'), glob('vision_module/template/*.png')),
        (os.path.join('lib', package_name, 'template_l_shape'), glob('vision_module/template_l_shape/*.png')),
        (os.path.join('lib', package_name, 'template2_shape'), glob('vision_module/template2_shape/*.png')),
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
            'realsense_rough = vision_module.realsense_rough:main',
            'realsenselib = vision_module.realsenselib:main',
            'detection_node = vision_module.detection_node:main',
            'screw_detector = vision_module.screw_detector:main',
            'l_shape_detector = vision_module.l_shape_detector:main',
            'icp_fitter = vision_module.icp_fitter:main',
        ],
    },
)
