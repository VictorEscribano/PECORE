from setuptools import setup
import os

package_name = 'non_linear_filters'

# Define the directory where the setup.py is located
here = os.path.abspath(os.path.dirname(__file__))

# List of YAML configuration files
yaml_config_files = [
    os.path.join('config', 'ekf_config.yaml'),
    # Add other YAML files here if necessary
]

# List of launch files
launch_files = [
    os.path.join('launch', 'ekf_gps.launch.py'),
    os.path.join('launch/include', 'jackal_localization.launch.py'),
    os.path.join('launch/include', 'jackal_sim.launch.py'),
    # Add other launch files here if necessary
]

# Setup function call
setup(
    name=package_name,
    version='0.0.0',
    packages=['non_linear_filters'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), yaml_config_files),
        (os.path.join('share', package_name, 'launch'), launch_files),
        # Include other non-python data files here as needed
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='victor',
    maintainer_email='victorescribanogarcia@gmail.com',
    description='A package for non-linear filtering in ROS2',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_ground_truth = non_linear_filters.odom_ground_truth:main',
            # Add other console_scripts here if necessary
        ],
    },
)
