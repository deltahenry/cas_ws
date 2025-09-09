from setuptools import setup

package_name = 'show_image_node'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Image display node for vision compensation system',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'show_image_node = show_image_node.show_image_node:main',
        ],
    },
)