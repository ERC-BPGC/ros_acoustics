from setuptools import setup

package_name = 'ros_acoustics'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + 'launch', ['launch/test1.launch.py']),
        ('share/' + package_name + 'launch', ['launch/test_rviz_launch.py']),
        ('share/' + package_name + 'launch', ['launch/basic_test_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tanmay Bhonsale',
    maintainer_email='tpb2k99@gmail.com',
    description='A package for quick and simple acoustics simulations for robots',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # publisher-subscribers defined here
            'odom_subscriber = ros_acoustics.'
        ],
    },
)