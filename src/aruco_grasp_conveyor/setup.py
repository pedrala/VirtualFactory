from setuptools import find_packages, setup

package_name = 'aruco_grasp_conveyor'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/aruco_grasp_conveyor']),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='booding',
    maintainer_email='booding@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gui_server = aruco_grasp_conveyor.gui_server:main',
            'hand_eye = aruco_grasp_conveyor.hand_eye:main',
            'learning_data = aruco_grasp_conveyor.learning_data:main',
            'status_test = aruco_grasp_conveyor.status_test:main',
            'send_email_1 = aruco_grasp_conveyor.send_email_1:main',
            'send_email_2 = aruco_grasp_conveyor.send_email_2:main',
            'manipulator = aruco_grasp_conveyor.manipulation_grip:main',
        ],
    },
)
