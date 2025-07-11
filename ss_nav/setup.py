from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ss_nav'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share/" + package_name), glob("launch/*_launch.py")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Johvany Gustave',
    maintainer_email='johvany.gustave@ipsa.fr',
    description='This package is used to define the navigation strategy of the UAV',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "mission.py = ss_nav.mission:main"
        ],
    },
)
