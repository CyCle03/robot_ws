from setuptools import find_packages, setup

package_name = 'my_turtlebot_pkg'

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
            'move_turtle_pub = my_turtlebot_pkg.move_turtle_pub:main',
            'detect_obstacle = my_turtlebot_pkg.detect_obstacle:main',
            'turtle_pose = my_turtlebot_pkg.turtle_pose:main',
        ],
    },
)
