from setuptools import find_packages, setup

package_name = 'collision_avoidance_system'

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
    maintainer='akhil_msi',
    maintainer_email='akhil_msi@temp.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'collision_monitor_control = collision_avoidance_system.collision_monitor_control:main'
        ],
    },
)
