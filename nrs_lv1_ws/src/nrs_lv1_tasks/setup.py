from setuptools import find_packages, setup

package_name = 'nrs_lv1_tasks'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lana',
    maintainer_email='lana@example.com',
    description='NRS LV1 zadaci 2 i 3',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'task2 = nrs_lv1_tasks.task2_cabinet:main',
            'task3_poses = nrs_lv1_tasks.task3_fingertip_pose:main',
        ],
    },
)