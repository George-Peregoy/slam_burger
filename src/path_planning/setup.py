from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'path_planning'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        (os.path.join('share', package_name, 'environments'),
         glob('environments/*pickle')),

        (os.path.join('share', package_name, 'qrcodes'),
         glob('qrcodes/*.png')),
         
    ],
    
    install_requires=[
        'setuptools',
        'numpy',
        'matplotlib',
        'imageio',
        'shapely',
        'qrcode',
        'pyzbar',
        'pillow'
        ],
    
    zip_safe=True,
    maintainer='gperegoy',
    maintainer_email='georgeperegoyr@gmail.com',
    description='',
    license='Apache-2.0',
    
    extras_require={
        'test': [
            'pytest',
        ],
    },
    
    entry_points={
        'console_scripts': [
            'path_publisher_2 = path_planning.pose_publisher_2:main',
            'qr_reader_node = path_planning.qr_reader_node:main',
            'path_publisher_1 = path_planning.pose_publisher_1:main',
            'path_publisher_1c = path_planning.pose_publisher_1c:main',
        ],
    },
)
