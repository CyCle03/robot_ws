from setuptools import find_packages, setup

package_name = 'my_turtle_controller'

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
    maintainer='penguin',
    maintainer_email='cherrybear03@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'simple_rotate = my_turtle_controller.simple_rotate:main',
            'pid_rotate = my_turtle_controller.pid_rotate:main',
            'pose_rotate_controller = my_turtle_controller.pose_rotate_controller:main',
            'gui_monitoring_control = my_turtle_controller.gui_monitoring_control:main',
        ],
    },
)
