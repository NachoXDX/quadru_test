from setuptools import find_packages, setup

package_name = 'quadruped_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    extras_require={'test': ['pytest']},
    zip_safe=True,
    maintainer='Nacho',
    maintainer_email='rubiolopedroignacio@gmail.com',
    description='Python nodes that move the Go2: posture (stand/lie), gaits, leg odometry.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'joy_actions_node = quadruped_control.joy_actions_node:main',
            'noisy_odom_node = quadruped_control.noisy_odom_node:main',
            'posture_node = quadruped_control.posture_node:main',
        ],
    },
)
