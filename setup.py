from setuptools import find_packages, setup

package_name = 'comprobo_project_1'

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
    maintainer='bwager',
    maintainer_email='bwager@olin.edu',
    description='RoboBehaviors and Finite State Machines Project',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop = comprobo_project_1.teleop:main',
            'figure_eight = comprobo_project_1.figure_eight:main',
            'wall_approach = comprobo_project_1.wall_approach:main'
        ],
    },
)
