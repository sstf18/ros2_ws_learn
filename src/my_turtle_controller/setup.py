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
    maintainer='seaian',
    maintainer_email='seaian@todo.todo',
    description='Turtlesim for ROS2 study',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        	'turtle_square = my_turtle_controller.turtle_square:main',
        	'turtle_circle = my_turtle_controller.turtle_circle:main',
        	'turtle_circle_param = my_turtle_controller.turtle_circle_param:main',
        	'turtle_circle_timer = my_turtle_controller.turtle_circle_timer:main',
        	
        ],
    },
)
