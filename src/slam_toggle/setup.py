from setuptools import find_packages, setup

package_name = 'slam_toggle'

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
    maintainer='jk',
    maintainer_email='jk@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'toggle_manager = slam_toggle.toggle_manager:main',
            'odom_switcher = slam_toggle.odom_switcher:main',
            'auto_toggle = slam_toggle.auto_toggle:main',
            'cmd_vel_mux = slam_toggle.cmd_vel_mux:main',
        ],
    },
)
