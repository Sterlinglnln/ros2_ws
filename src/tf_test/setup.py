from setuptools import find_packages, setup

package_name = 'tf_test'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='larry',
    maintainer_email='fpnf97@outlook.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'staticframepub = tf_test.staticframepub:main',
            'frame_tf2_pub = tf_test.framepub:main',
            'frame_tf2_sub = tf_test.framesub:main',
        ],
    },
)
