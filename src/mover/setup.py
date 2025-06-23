from setuptools import setup

package_name = 'mover'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='Move robot to a goal using /cmd_vel and /odom',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nodd = mover.nodd:main',  # command: ros2 run mover nodd
            'robot1_follow_robot2 = mover.robot1_follow_robot2:main', 
       ],
    },
)

