from setuptools import find_packages, setup

package_name = 'gl5_localization'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/scan_matcher.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dhkim',
    maintainer_email='dhkim@example.com',
    description='Keyframe ICP scan matcher for the GL5/GL3 workshop',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'gl5_scan_matcher = gl5_localization.scan_matcher_node:main',
        ],
    },
)
