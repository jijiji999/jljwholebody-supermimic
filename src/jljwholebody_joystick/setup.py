from glob import glob
from setuptools import find_packages, setup


package_name = "jljwholebody_joystick"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="rx",
    maintainer_email="rx@example.com",
    description="Joystick driver node for the JLJ whole-body humanoid platform.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "joystick_node = jljwholebody_joystick.joystick_node:main",
        ],
    },
)
