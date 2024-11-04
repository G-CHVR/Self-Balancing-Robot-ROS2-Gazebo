import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'self_balancing_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "worlds"), glob("worlds/*.world")),
        (os.path.join("share", package_name, "model"), glob("model/*.urdf")),
        (os.path.join("share", package_name, "model"), glob("model/*.xacro")),
        (os.path.join("share", package_name, "model"), glob("model/*.gazebo")),
        (os.path.join("share", package_name, "model"), glob("model/*.yml"))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gch',
    maintainer_email='glen.chevalier@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pid_controller=self_balancing_robot.pid_controller:main'
        ],
    },
)
