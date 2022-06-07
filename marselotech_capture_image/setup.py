from setuptools import setup
import os 
from glob import glob 

package_name = 'marselotech_capture_image'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'launch'), glob('marselotech_capture_image/*.py')),
        (os.path.join('share', package_name, 'launch'), glob('marselotech_capture_image/*.json')),
        (os.path.join('lib', package_name , 'launch'), glob('marselotech_capture_image/*.json')),
        (os.path.join('share', package_name, 'launch'), glob('marselotech_capture_image/clasificadores/*.xml')),
        (os.path.join('lib', package_name), glob('marselotech_capture_image/clasificadores/*.xml'))



    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='leire',
    maintainer_email='lvilmar1@epsg.upv.es',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [ 
            'capture_image=marselotech_capture_image.capturar:main',
            'capture_person=marselotech_capture_image.capturar_personas:main',
            'capture_green=marselotech_capture_image.detectar_verde:main',
            'capture_face=marselotech_capture_image.capturar_caras:main',
            'capture_person_real=marselotech_capture_image.capturar_personas_real:main',
            'capture_green_real=marselotech_capture_image.detectar_verde_real:main',
            'capture_face_real=marselotech_capture_image.capturar_caras_real:main',
            'capture_image_real=marselotech_capture_image.capturar_real:main',
            'detection_server = marselotech_capture_image.detection_server:main',
            'detection_server_sim = marselotech_capture_image.detection_server_sim:main',
            'save_image_service_sim = marselotech_capture_image.save_image_server_sim:main',
            'save_image_service = marselotech_capture_image.save_image_server:main'

        ],
    },
)
