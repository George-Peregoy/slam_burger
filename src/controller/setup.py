from setuptools import find_packages, setup

package_name = 'controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gperegoy',
    maintainer_email='georgeperegoyr@gmail.com',
    description='Controller package for TurtleBot3 BurgerBots',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',

        ],
    },
    entry_points={
        'console_scripts': [
            'controller_node = controller.robot_controller:main',
            'controller_node_c = controller.robot_controller_c:main',
            'controller_node_1b = controller.robot_controller_1b:main'
        ],
    },
)
