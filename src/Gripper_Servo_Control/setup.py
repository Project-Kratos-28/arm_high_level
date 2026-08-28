from setuptools import find_packages, setup

package_name = 'Gripper_Servo_Control'

setup(
    name=package_name,
    version='0.0.0',

    packages=find_packages(exclude=['test']),

    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
    ],

    install_requires=[
        'setuptools',
    ],

    zip_safe=True,

    maintainer='propmaster49',
    maintainer_email='propmaster49@gmail.com',

    description='Gripper servo control',
    license='TODO: License declaration',

    extras_require={
        'test': [
            'pytest',
        ],
    },

    entry_points={
        'console_scripts': [
            'servo_position = Gripper_Servo_Control.Servo_Position_Control:main',
            'servo_monitor = Gripper_Servo_Control.Servo_Angle_Load_Subscriber:main',
        ],
    },
)