from setuptools import find_packages, setup

package_name = 'px4_gui_ctrl'

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
    maintainer='hk',
    maintainer_email='khg950520@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'gui_node = px4_gui_ctrl.gui_node:main',
            'auto_painter_node = px4_gui_ctrl.auto_painter:main',
            'manual_controller_node = px4_gui_ctrl.manual_controller:main',
            'drone_teleop_node = px4_gui_ctrl.drone_teleop:main',
        ],
    },
)
