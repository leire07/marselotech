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
        (os.path.join('share', package_name, 'config'), glob('config/*.pgm')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'launch'), glob('marselotech_my_nav2_system/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.lua')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.xml')) 

        
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
            'initial_pose_pub = marselotech_my_nav2_system.initial_pose_pub:main', #añadir
            'action_server = marselotech_my_nav2_system.action_server:main',
            'nav_to_pose = marselotech_my_nav2_system.nav_to_pose:main',   # incluir
            'waypoint_follower = marselotech_my_nav2_system.waypoint_follower:main'

        ],
    },
)
