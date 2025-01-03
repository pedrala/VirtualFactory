from setuptools import find_packages, setup

package_name = 'virtual_factory'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/virtual_factory']),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='viator',
    maintainer_email='pedralation@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gui = virtual_factory.server.gui_server:main',
            'aruco = virtual_factory.aruco_marker.aruco_detection:main',
            'amr = virtual_factory.amr.amr_node:main',
            'manipulator = virtual_factory.manipulator.manipulation_grip:main',
        ],
    },
)
