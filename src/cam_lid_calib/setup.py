from setuptools import find_packages, setup

package_name = 'cam_lid_calib'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'sensor_msgs',
        'cv_bridge',
        'image_transport',
        'interactive_markers',
    ],
    zip_safe=True,
    maintainer='amr2',
    maintainer_email='amr2@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'laser_image_subscriber = cam_lid_calib.laser_image_subscriber:main',
            'image_click_node = cam_lid_calib.image_click_node:main',
            'interactive_laserscan = cam_lid_calib.interactive_laserscan:main',
        ],
    },
)
