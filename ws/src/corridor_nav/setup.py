from glob import glob
from setuptools import setup

package_name = 'corridor_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.geojson')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Göksel Özgür',
    maintainer_email='gokselozgur@novit.ai',
    description='Pure-pursuit follower, patrol action, geofence monitor.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'waypoint_follower = corridor_nav.waypoint_follower:main',
            'geofence_monitor = corridor_nav.geofence_monitor:main',
            'send_path = corridor_nav.send_path:main',
        ],
    },
)
