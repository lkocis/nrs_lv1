from glob import glob
from setuptools import setup

setup(
    name='cabinet_generator', version='0.1.0', packages=['cabinet_generator'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/cabinet_generator']),
        ('share/cabinet_generator', ['package.xml']),
        ('share/cabinet_generator/launch', glob('launch/*.launch.py')),
        ('share/cabinet_generator/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'], tests_require=['pytest'],
    maintainer='Valentin Simundic', maintainer_email='valentin.simundic@ferit.hr',
    description='NRS lab exercise', license='Apache-2.0',
    entry_points={'console_scripts': ['spawn_cabinet = cabinet_generator.spawn_cabinet:main']},
)
