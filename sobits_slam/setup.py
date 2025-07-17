# from setuptools import find_packages, setup
import os
from glob import glob
from setuptools import setup

package_name = 'sobits_slam'

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'location'), glob('location/*.yaml')),
        (os.path.join('share', package_name, 'map'), glob('map/*')),
        (os.path.join('share', package_name, 'img'), glob('img/*')),
        (os.path.join('share', package_name, 'param', 'sobit_pro'), glob('param/sobit_pro/*.yaml')),
        (os.path.join('share', package_name, 'param', 'sobit_edu'), glob('param/sobit_edu/*.yaml')),
        (os.path.join('share', package_name, 'param', 'sobit_mini'), glob('param/sobit_mini/*.yaml')),
        (os.path.join('share', package_name, 'param', 'sobit_light'), glob('param/sobit_light/*.yaml')),
        (os.path.join('share', package_name, 'param', 'hsr_sim'), glob('param/hsr_sim/*.yaml')),
        (os.path.join('share', package_name, 'param', 'hsrb_robot'), glob('param/hsrb_robot/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Fumiya Ono',
    maintainer_email='fumiyaono.choi@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    # tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sobits_map_saver = sobits_slam.sobits_map_saver:main',
            'location_setting = sobits_slam.location_setting:main',
        ],
    },
)
