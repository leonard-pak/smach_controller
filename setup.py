from setuptools import setup

package_name = 'smach_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Leonard pak',
    maintainer_email='leopak2000@gmail.com',
    description='Controller of state machine (smach)',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'start = smach_controller.start:main'
        ],
    },
)
