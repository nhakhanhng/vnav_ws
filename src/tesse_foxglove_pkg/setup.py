from glob import glob
from setuptools import setup

package_name = 'tesse_foxglove_pkg'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.json')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vnav',
    maintainer_email='vnav@mit.edu',
    description='Foxglove visualization of TESSE quadrotor state and trajectory',
    license='MIT',
    entry_points={
        'console_scripts': [
            'tesse_viz_publisher = tesse_foxglove_pkg.viz_publisher:main',
        ],
    },
)
