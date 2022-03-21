from setuptools import setup
import os 
from glob import glob 

package_name = 'marselotech_my_nav2_system'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'map'), glob('map/*.pgm')),
        (os.path.join('share', package_name, 'map'), glob('map/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.lua')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')) 
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='asperez@upv.es',
    maintainer_email='asperez@upv.es@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'initial_pose_pub = marselotech_my_nav2_system.initial_pose_pub:main' #añadir
        ],
    },
)
