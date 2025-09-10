from setuptools import find_packages, setup

package_name = 'turbopi_base'

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
    maintainer='Garegin',
    maintainer_email='garegin.ma@gmail.com',
    description="TurboPi base driver for mecanum using HiWonder SDK.",
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drive = turbopi_base.drive_node:main',
        ],
    },
)
