from setuptools import find_packages, setup
from glob import glob

package_name = 'turtle_exercise'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
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
            'draw_spiral = turtle_exercise.draw_spiral:main',
            'spawn = turtle_exercise.spawn:main',
            'action_move = turtle_exercise.action_move:main',
            'random_walk = turtle_exercise.random_walk:main',
            'clear_turtle = turtle_exercise.clear_turtle:main',
            'spawn_cmd = turtle_exercise.spawn_cmd:main',
            'set_pen = turtle_exercise.set_pen:main',
        ],
    },
)
