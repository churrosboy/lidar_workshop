from setuptools import find_packages, setup

package_name = 'gl5_detection'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/obstacles.yaml', 'config/obstacles_step1.yaml',
                                                'config/obstacles_step2.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dhkim',
    maintainer_email='dhkim@example.com',
    description='GL5 region editing, obstacle detection and tracking',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'gl5_obstacle_node = gl5_detection.gl5_obstacle_node:main',
        ],
    },
)
