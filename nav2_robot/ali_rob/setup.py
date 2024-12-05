from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ali_rob'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'world'), glob('world/*.world')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ali',
    maintainer_email='ali@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control = ali_rob.control:main',
            'line_following = ali_rob.line_following:main',
            'combine = ali_rob.combine:main',
            'obstical_avoidence = ali_rob.obstical_avoidence:main',
            "mpu6050 = ali_rob.mpu6050:main",
            "gyro_mpu6050 = ali_rob.gyro_mpu6050:main"
        ],
    },
)

