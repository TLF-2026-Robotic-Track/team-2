from setuptools import find_packages, setup

package_name = 'master_launch'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # One line per launch file. Add another line if you add another one.
        ('share/' + package_name + '/launch', [
            'launch/keyboard.launch.xml',
            'launch/demo.launch.xml',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='Launch files that start several nodes at once',
    license='GPLv3',
)
