from glob import glob
from setuptools import find_packages, setup

package_name = 'gazebo_quadrotor_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*')),
        ('share/' + package_name + '/urdf', glob('urdf/*')),
        ('share/' + package_name + '/worlds', glob('worlds/*')),
        ('share/' + package_name + '/config', glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vnav',
    maintainer_email='vnav@mit.edu',
    description='Gazebo Classic quadrotor simulator for VNAV Lab 3',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'quadrotor_bridge = gazebo_quadrotor_pkg.quadrotor_bridge:main',
            'lab3_viz_publisher = gazebo_quadrotor_pkg.viz_publisher:main',
        ],
    },
)
