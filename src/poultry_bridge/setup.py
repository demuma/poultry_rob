from setuptools import setup

package_name = "poultry_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        # ament index resource – required for 'ros2 pkg' to find the package
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/bridge.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Developer",
    maintainer_email="dev@example.com",
    description="UDS ↔ ROS 2 bridge for poultry farm",
    license="MIT",
    entry_points={
        "console_scripts": [
            "uds_bridge_node = poultry_bridge.uds_bridge_node:main",
            "uds_client      = poultry_bridge.uds_client:main",
            "uds_server      = poultry_bridge.uds_server:main",
        ],
    },
)
