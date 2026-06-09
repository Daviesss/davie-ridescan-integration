from setuptools import find_packages, setup

package_name = 'ridescan_ros2_bridge'

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
    maintainer='ubuntu',
    maintainer_email='oguns53@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        'ride_scan_ros2_bridge= ridescan_ros2_bridge.ridescan_bridge_node:main',
        'ride_scan_test_ros2_bridge= ridescan_ros2_bridge.ridescan_test_bridge_node:main',
        'ride_scan_diagnostics= ridescan_ros2_bridge.ridescan_diagnostics:main',
        'ride_scan_safety_monitor= ridescan_ros2_bridge.ridescan_safety_node:main',
        'ride_scan_risk_plot= ridescan_ros2_bridge.ridescan_risk_plot:main',
        'ride_scan_csv_node= ridescan_ros2_bridge.ridescan_bride_csv_node:main',
        'ride_scan_csv_diagnostics= ridescan_ros2_bridge.ridescan_csv_diagnostics:main',
        'way_point_follower_node= ridescan_ros2_bridge.way_point_follower:main', # way point follower node script , robot navigates to the same waypoint in a map or environment repeatedly, often for testing or demonstration purposes.
        ],
    },
)
