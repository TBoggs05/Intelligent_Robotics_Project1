from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'reactive_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
	#opens non python files on launch
	(os.path.join('share', package_name, 'launch'), glob('launch/*.py')), (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),  
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='card0181',
    maintainer_email='luis.f.cardenas-1@ou.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
