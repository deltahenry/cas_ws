from setuptools import setup

package_name = 'my_flexbe_behaviors'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/behaviors', ['behaviors/example_behavior.xml']),  # 加入你的 behavior xml
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='你的名字',
    maintainer_email='你的信箱',
    description='My FlexBE behavior package',
    license='MIT',
    entry_points={
        'console_scripts': [],
    },
)
