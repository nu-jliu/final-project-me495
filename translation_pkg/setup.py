from setuptools import find_packages, setup
import glob
import os

package_name = 'translation_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/worlds', glob.glob('worlds/*.world')),
        ('share/' + package_name + '/launch', glob.glob('launch/*launch.*')),
        ('share/' + package_name + '/config', glob.glob('config/*')),
        ('share/' + package_name + '/urdf', glob.glob('urdf/*.xacro')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='damien',
    maintainer_email='damienkoh2025@u.northwestern.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'translator_node = translation_pkg.translator_node:main'
        ],
    },
)
