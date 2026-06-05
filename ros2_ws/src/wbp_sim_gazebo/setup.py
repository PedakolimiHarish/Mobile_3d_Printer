from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'wbp_sim_gazebo'

def collect_files(dir_path):
    files = []
    for root, _, filenames in os.walk(dir_path):
        for f in filenames:
            files.append(os.path.join(root, f))
    return files

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # ROS package index
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),

        # package.xml
        ('share/' + package_name, ['package.xml']),

        # Gazebo worlds
        ('share/' + package_name + '/worlds', glob('worlds/*.sdf')),

        # Gazebo models (recursive)
        *[
            (
                'share/' + package_name + '/' + os.path.dirname(path),
                [path]
            )
            for path in collect_files('gazebo_models')
        ],
        
        # Config files (recursive)
        *[
            (
                'share/' + package_name + '/' + os.path.dirname(path),
                [path]
            )
            for path in collect_files('config')
        ],

        # Launch files
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='guru',
    maintainer_email='pedakolimi.harish@gmail.com',
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
