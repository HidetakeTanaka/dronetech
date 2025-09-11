from setuptools import find_packages, setup
from glob import glob   

package_name = 'project_report'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),  # keeps launch files
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eolab',
    maintainer_email='tanaka.hidetake@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'landing_subscriber = project_report.landing_subscriber:main',
            'landing_controller = project_report.landing_controller:main',
            'marker_to_error = project_report.marker_to_error:main',
            'echo_manual_once  = project_report.echo_manual_once:main',  
        ],
    },
)

