from setuptools import setup, find_packages

package_name = 'pathtracking'

setup(
    name=package_name,
    version='0.0.1',
    packages=['scripts'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Path tracking package for Diablo robot using PD control',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'diablotrack = scripts.diablo_track:main',
            'testtrack = scripts.test_track:main',
            'wheelodom = scripts.wheel_odom:main',
        ],
    },
)
