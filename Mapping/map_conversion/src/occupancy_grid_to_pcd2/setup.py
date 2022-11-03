from setuptools import setup

package_name = 'occupancy_grid_to_pcd2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='volpe',
    maintainer='volpe',
    maintainer_email='philip.azeredo@dot.gov',
    description='Converts Occupancy Grid Maps to PointCloud2 Format for HD Mapping',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "print_occ_grid_csv = occupancy_grid_to_pcd2.numpy_array_printer:main",
            "occ_grid_to_pcd2 = occupancy_grid_to_pcd2.numpy_occ_to_pcd2:main"
        ],
    },
)
