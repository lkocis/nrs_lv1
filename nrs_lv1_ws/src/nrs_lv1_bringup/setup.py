from glob import glob
from setuptools import setup

setup(
    name='nrs_lv1_bringup', version='0.1.0', packages=['nrs_lv1_bringup'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/nrs_lv1_bringup']),
        ('share/nrs_lv1_bringup', ['package.xml']),
        ('share/nrs_lv1_bringup/launch', glob('launch/*.launch.py')),
        ('share/nrs_lv1_bringup/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    maintainer='Valentin Simundic', maintainer_email='valentin.simundic@ferit.hr',
    description='NRS lab exercise', license='Apache-2.0',
    entry_points={'console_scripts': []},
)
