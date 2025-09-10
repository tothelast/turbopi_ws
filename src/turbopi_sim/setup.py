from setuptools import setup

package_name = 'turbopi_sim'

setup(
    name=package_name,
    version='0.0.1',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/turbopi_sim.launch.py']),
        ('share/' + package_name + '/urdf', ['urdf/turbopi.urdf']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Garegin',
    maintainer_email='you@example.com',
    description='TurboPi simulation in Gazebo Harmonic',
    license='MIT',
    entry_points={},
)
