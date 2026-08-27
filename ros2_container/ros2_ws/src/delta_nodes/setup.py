from setuptools import setup

package_name = 'delta_nodes'

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
    maintainer='Sreeram',
    maintainer_email='sreeram3927@zohomail.in',
    description='Jetson-side ROS2 nodes: ESP32/Arduino serial bridges, camera target ingest, frame transform, target selection, command arbitration, frontend websocket bridge',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'manipulator_bridge = delta_nodes.manipulator_bridge_node:main',
            'mobile_base_bridge = delta_nodes.mobile_base_bridge_node:main',
            'camera_target_bridge = delta_nodes.camera_target_bridge_node:main',
            'target_transform = delta_nodes.target_transform_node:main',
            'target_selector = delta_nodes.target_selector_node:main',
            'command_arbiter = delta_nodes.command_arbiter_node:main',
            'frontend_bridge = delta_nodes.frontend_bridge_node:main',
        ],
    },
)
