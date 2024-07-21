from setuptools import find_packages, setup

package_name = 'task2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy', 'opencv-python', 'mediapipe'],
    zip_safe=True,
    maintainer='afham',
    maintainer_email='afham@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'task2_node = task2.task2_node:main' ,
            'task2_subscriber = task2.task2_subscriber:main',
        ],
    },
)
