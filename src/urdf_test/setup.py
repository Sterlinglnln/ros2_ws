from setuptools import find_packages, setup
from glob import glob

package_name = 'urdf_test'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name+ '/urdf', glob('urdf/*.urdf')),
        ('share/' + package_name+ '/urdf', glob('urdf/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='larry',
    maintainer_email='fpnf97@outlook.com',
    description='TODO: Package description',
    license='MIT',
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
