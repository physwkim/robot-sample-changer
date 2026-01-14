from setuptools import find_packages, setup

package_name = 'robot_gui'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'pyepics',
        'silx',
        'PyQt6',
        'numpy',
    ],
    zip_safe=True,
    maintainer='stevek',
    maintainer_email='stevek@todo.todo',
    description='PyQt6/silx GUI for EPICS Robot Control',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_control_gui = robot_gui.main:main',
        ],
    },
)
