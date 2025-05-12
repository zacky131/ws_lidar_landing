from setuptools import find_packages, setup

package_name = 'px4_keyboard_control'

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
    maintainer='root',
    maintainer_email='aktama8998@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'px4_keyboard_handler = px4_keyboard_control.keyboard:main',
            'px4_offboard_control = px4_keyboard_control.offboard:main',
            'px4_servo_control = px4_keyboard_control.servo_downward:main',
        ],
    },
)
