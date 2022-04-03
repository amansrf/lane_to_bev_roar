from setuptools import setup

package_name = 'lane_to_bev_roar'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='roar',
    maintainer_email='amansaraf99@gmail.com',
    description='Bird\'s Eye View Maker',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lane_mask_publisher = lane_to_bev_roar.lane_detect_mask:main',
            'bev_publisher = lane_to_bev_roar.pcl_to_bev:main',
        ],
    },
)
