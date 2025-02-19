import os
from glob import glob
from setuptools import find_packages, setup

package_name = "amzsim_rviz_handler"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.py")),
        ),
        (
            os.path.join("share", package_name, "rviz"),
            glob(os.path.join("rviz", "*config.rviz")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ronan",
    maintainer_email="ronan.tanios@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "edit_rviz.py = amzsim_rviz_handler.edit_rviz:main",
        ],
    },
)
