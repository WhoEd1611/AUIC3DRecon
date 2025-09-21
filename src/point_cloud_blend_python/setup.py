from setuptools import find_packages, setup

package_name = 'point_cloud_blend_python'

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
    maintainer='edric',
    maintainer_email='edriclay2002@outlook.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "serial_node = point_cloud_blend_python.arduinoWriter:main",
            "rotation_node = point_cloud_blend_python.rotationTester:main"
        ],
    },
)
