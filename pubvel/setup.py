from setuptools import find_packages, setup

package_name = 'pubvel'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/velocity_publisher_launch.py'])  # Declarar launch
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='victor.escribano.garcia',
    maintainer_email='victor.escribano.garcia@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'pubvel = pubvel.pubvel:main',
        'simple_service_requester = pubvel.simple_service_requester:main',
        'pid_pubvel = pubvel.pid_pubvel:main'
        ],
    },
)
