from setuptools import setup, find_packages
from glob import glob
import os

package_name = "graph_nav2"

setup(
    name=package_name,
    version="0.0.1",

    # ① pick up graph_nav2 *and* graph_nav2.TrafficCommunication.*
    packages=find_packages(include=[
                           package_name, f"{package_name}.*",  'TrafficCommunication', 'TrafficCommunication.*',]),

    # ② keep only real data assets in data_files
    data_files=[
        ("share/ament_index/resource_index/packages",
         [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (os.path.join("share", package_name, "launch"),  glob("launch/*.py")),
        (os.path.join("share", package_name, "config"),  glob("config/*.yaml")),
        (os.path.join("share", package_name, "maps"),    glob("maps/*")),
        (os.path.join("share", package_name, "srv"),     glob("srv/*.srv")),
        # only *non-Python* extras, e.g. the PEM file:
        (os.path.join("share", package_name, "useful"),
         ["graph_nav2/TrafficCommunication/useful/publickey_server_test.pem"]),
    ],

    install_requires=[
        "setuptools",
        "networkx",
        "pyproj",
        "rosidl_default_runtime",
        "graph_nav2_interfaces",
    ],
    zip_safe=True,
    maintainer="Your Name",
    maintainer_email="you@example.com",
    description="Waypoint navigation from GraphML using Nav2.",
    license="Apache-2.0",
    tests_require=['pytest'],
    entry_points={
        "console_scripts": [
            "graph_loader       = graph_nav2.graph_loader:main",
            "gps_localizer      = graph_nav2.gps_localizer:main",
            "waypoint_navigator = graph_nav2.waypoint_navigator:main",
            "motor_controller   = graph_nav2.motor_controller:main",
            "traffic_comm       = graph_nav2.traffic_communication_node:main",
        ],
    },
)
