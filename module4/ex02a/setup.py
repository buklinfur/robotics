from setuptools import find_packages, setup

package_name = 'ex02a'

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/turtle_tf2_demo.launch.py"]),
        ("share/" + package_name + "/rviz", ["rviz/carrot.rviz"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="buklinfur",
    maintainer_email="buklinfur@outlook.com",
    description="TODO: Package description",
    license='Apache-2.0',
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "turtle_tf2_broadcaster = ex02.turtle_tf2_broadcaster:main",
            "turtle_tf2_listener = ex02.turtle_tf2_listener:main",
            "carrot_broadcaster = ex02.carrot_broadcaster:main",
        ],
    },
)
