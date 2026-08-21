from setuptools import find_packages, setup

package_name = 'blinker'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='Blinks the LEDs of the Duckiebot',
    license='GPLv3',
    entry_points={
        'console_scripts': [
            'blinker = blinker.blinker:main',
        ],
    },
)
