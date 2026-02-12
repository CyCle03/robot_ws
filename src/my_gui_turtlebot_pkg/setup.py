from setuptools import find_packages, setup
from glob import glob

package_name = 'my_gui_turtlebot_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
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
            'move_turtlebot_pub = my_gui_turtlebot_pkg.move_turtlebot_pub:main',
            'turtlebot_move_con = my_gui_turtlebot_pkg.turtlebot_move_con:main',
            'patrol_action_server = my_gui_turtlebot_pkg.patrol_action_server:main',
            'detect_obstacle = my_gui_turtlebot_pkg.detect_obstacle:main',
            'cmd_vel_arbiter = my_gui_turtlebot_pkg.cmd_vel_arbiter:main',
        ],
    },
)
