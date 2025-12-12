from setuptools import find_packages, setup

package_name = 'action_cleaning_robot'

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
    maintainer='buklinfur',
    maintainer_email='buklinfur@outlook.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "cleaning_action_server = action_cleaning_robot.cleaning_action_server:main",
            "cleaning_action_client = action_cleaning_robot.cleaning_action_client:main",
        ],
    },
)
