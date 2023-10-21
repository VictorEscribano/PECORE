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
        ('share/' + package_name + '/launch', ['launch/practicum1.launch.py']) 
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
        'aruco_approach = visual_servoing_P1.aruco_approach:main'
        ],
    },
)
