from setuptools import find_packages, setup

package_name = "monitoring_display"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(include=[package_name, package_name + ".*"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Jonas Ohnemus",
    maintainer_email="johnemus@ethz.ch",
    description="Terminal / Command Line Display for Monitoring",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "monitor = monitoring_display.node_start:main"  # Ensure this matches the actual filename
        ],
    },
)
