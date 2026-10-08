from glob import glob

from setuptools import setup

package_name = 'kumawatch_ai'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/data', glob('data/*')),
        ('share/' + package_name + '/models', glob('models/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='KumaWatch Patrol maintainers',
    maintainer_email='kumawatch.patrol@example.com',
    description='KumaWatch Patrol: bear-encounter risk estimation',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'risk_predictor = kumawatch_ai.risk_predictor:main',
            'bear_detector = kumawatch_ai.bear_detector:main',
        ],
    },
)
