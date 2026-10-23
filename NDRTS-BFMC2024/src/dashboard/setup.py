# src/dashboard/setup.py
from setuptools import setup, find_packages
from pathlib import Path
from glob import glob
import os

package_name = "dashboard"
here = Path(__file__).parent

# every FILE inside webapp/dashboard-ui
webapp_root = here / "webapp" / "dashboard-ui"
webapp_files = [
    str(p) for p in here.joinpath('webapp').rglob('*')
    if p.is_file() and 'node_modules' not in str(p) and 'dist' not in str(p)
]


setup(
    name=package_name,
    version="0.0.0",
    # don’t treat launch/ as Python
    packages=find_packages(exclude=["test", "launch"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
         [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch",
         [str(p) for p in here.joinpath("launch").glob("*.py")]),

        # ↓ install webapp files under the *same* dir name
        (f"share/{package_name}/webapp/dashboard-ui", webapp_files),

        # Node-JS bridge
        (f"share/{package_name}/server", glob("server/*.js")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ndrts",
    maintainer_email="ndrts@todo.todo",
    description="Dashboard with Vite front-end and ROS2 websocket bridge",
    license="MIT",
    tests_require=['pytest'],
    entry_points={
        "console_scripts": [
            # (no Python executables for this package)
        ],
    },
)
