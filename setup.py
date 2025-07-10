from setuptools import find_packages, setup

package_name = 'arcs_cohort_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name + '/launch', [
            'launch/simulation_bringup.launch.py']),
        ('share/' + package_name + '/launch', [
            'launch/multi_rover_sim_bringup.launch.py']),
        ('share/' + package_name + '/config', [
            'config/two_rovers.yaml']),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Barry Ridge',
    maintainer_email='barry.ridge@csun.edu',
    description='A bringup package containing launchers for the ARCS CoHORT project.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
