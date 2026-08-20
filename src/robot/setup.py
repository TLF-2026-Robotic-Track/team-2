from setuptools import find_packages, setup

package_name = 'robot'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/launch.xml']),
        # One line per node. Add another line if you add another node file.
        ('lib/' + package_name, ['robot/main.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='Receives commands and drives the wheels and LEDs of the Duckiebot',
    license='GPLv3',
)
