from setuptools import find_packages, setup

package_name = 'visual_servoing_P1'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    py_modules=['visual_servoing_P1.transformations'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/pid_control_p1.launch.py']),
        ('share/' + package_name + '/launch', ['launch/ros_nav_stack_p1.launch.py']),
        ('share/' + package_name + '/launch', ['launch/visual_servoing.launch.py'])  
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='victor',
    maintainer_email='victorescribanogarcia@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'generate_map_frame = visual_servoing_P1.generate_map_frame:main',
        'aruco_approach = visual_servoing_P1.aruco_approach:main',
        'robot_vel_controller = visual_servoing_P1.robot_vel_controller:main',
        'pbvs_vel_controller = visual_servoing_P1.pbvs_vel_controller:main'
        ],
    },
)
