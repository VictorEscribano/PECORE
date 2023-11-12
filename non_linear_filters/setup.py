from setuptools import find_packages, setup

package_name = 'non_linear_filters'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/ekf_gps.launch.py']),
        ('share/' + package_name + '/launch/include', ['launch/include/jackal_localization.launch.py']),
        ('share/' + package_name + '/launch/include', ['launch/include/jackal_sim.launch.py'])
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
         'odom_ground_truth = non_linear_filters.odom_ground_truth:main'
        ],
    },
)
