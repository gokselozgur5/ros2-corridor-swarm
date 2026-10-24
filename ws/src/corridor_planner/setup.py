from setuptools import setup

package_name = 'corridor_planner'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Göksel Özgür',
    maintainer_email='gokselozgur@novit.ai',
    description='Claude mission planner for the corridor fleet.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_planner = corridor_planner.mission_planner:main',
        ],
    },
)
