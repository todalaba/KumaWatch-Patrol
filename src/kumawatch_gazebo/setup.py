from glob import glob

from setuptools import setup

package_name = 'kumawatch_gazebo'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/worlds', glob('worlds/*.world')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='KumaWatch Patrol maintainers',
    maintainer_email='kumawatch.patrol@example.com',
    description='KumaWatch Patrol: Gazebo world',
    license='Apache-2.0',
    tests_require=['pytest'],
)
