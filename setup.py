from glob import glob
from os import path
from setuptools import find_packages, setup

package_name = 'mess2_logger_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (path.join("share", package_name, "launch"), glob(path.join("launch", "*launch.[pxy][yma]*"))),
        (path.join("share", package_name, "config"), glob(path.join("config", "*"))),
    ],
    py_modules=[
        'src.log_topics_to_csvs',
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mess2',
    maintainer_email='marinarasauced@outlook.com',
    description='ROS2 package for logging messages on topics directly to .csv files.',
    license='Apache 2.0',
    entry_points={
        'console_scripts': [
            'log_topics_to_csvs = src.log_topics_to_csvs:main',
        ],
    },
)
