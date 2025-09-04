from setuptools import find_packages, setup

package_name = 'project_report'

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
    maintainer='eolab',
    maintainer_email='tanaka.hidetake@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
<<<<<<< HEAD
        'landing_subscriber = project_report.landing_subscriber:main',
=======
            'landing_subscriber = project_report.landing_subscriber:main',
            'landing_controller = project_report.landing_controller:main',
>>>>>>> 3644b06 (Created landing_controller and landing_subscriber but stuck)
        ],
    },
)
