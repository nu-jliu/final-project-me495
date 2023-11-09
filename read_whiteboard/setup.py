from setuptools import find_packages, setup

package_name = 'read_whiteboard'

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
    maintainer='megan',
    maintainer_email='meganblack2027@u.northwestern.edu',
    description='Reads written text from a whiteboard'
    license='MIT=['pytest'],
    entry_points={
        'console_scripts': [
            'computer_vision = read_whiteboard.computer_vision:entry_point'
        ],
    },
)
