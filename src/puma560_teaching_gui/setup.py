from setuptools import setup

package_name = 'puma560_teaching_gui'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='allen',
    maintainer_email='allen891221iris@gmail.com',
    description='PUMA560 正向運動學教學 GUI（PyQt5 + rclpy）與運動學模組',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teaching_gui = puma560_teaching_gui.gui_node:main',
        ],
    },
)
