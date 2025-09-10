from setuptools import setup, find_packages

setup(
    name="hiwonder_turbopi_sdk",
    version="0.1.0",
    description="HiWonder TurboPi SDK, packaged for ROS 2.",
    packages=find_packages(include=["HiwonderSDK", "HiwonderSDK.*"]),
    package_data={"HiwonderSDK": ["*.yaml", "*.yml", "*.npz", "*.jpg"]},
    install_requires=[],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/hiwonder_turbopi_sdk"]),
        ("share/hiwonder_turbopi_sdk", ["package.xml"]),
    ],
    zip_safe=False,
)
