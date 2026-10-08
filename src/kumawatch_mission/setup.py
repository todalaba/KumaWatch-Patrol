from glob import glob

from setuptools import setup

package_name = 'kumawatch_mission'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='KumaWatch Patrol maintainers',
    maintainer_email='kumawatch.patrol@example.com',
    description='KumaWatch Patrol: risk-aware patrol mission',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_planner = kumawatch_mission.mission_planner:main',
            'bear_proximity_monitor = kumawatch_mission.bear_proximity_monitor:main',
            'fake_nav2 = kumawatch_mission.fake_nav2:main',
            'image_recorder = kumawatch_mission.image_recorder:main',
            'mission_console = kumawatch_mission.mission_console:main',
            'chase_camera = kumawatch_mission.chase_camera:main',
        ],
    },
)
