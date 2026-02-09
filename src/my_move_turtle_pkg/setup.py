from setuptools import find_packages, setup

package_name = 'my_move_turtle_pkg'

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
  maintainer_email='penguin@todo.todo',
  description='TODO: Package description',
  license='TODO: License declaration',
  extras_require={
    'test': [
      'pytest',
    ],
  },
  entry_points={
    'console_scripts': [
      'move_turtle_pub = my_move_turtle_pkg.move_turtle_pub:main',
      'turtle_cmd_and_pose = my_move_turtle_pkg.turtle_cmd_and_pose:main',
      'my_service_server = my_move_turtle_pkg.my_service_server:main',
      'dist_turtle_action_server = my_move_turtle_pkg.dist_turtle_action_server:main',
      'my_subscriber = my_move_turtle_pkg.my_subscriber:main',
      'my_multi_thread = my_move_turtle_pkg.my_multi_thread:main',
      'turtle_move_con = my_move_turtle_pkg.turtle_move_con:main',
    ],
  },
)
