from setuptools import setup

package_name = 'yolo_crater_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name + '/launch', ['launch/yolo_crater_detector.launch.py']),
        ('share/' + package_name + '/models', ['models/best.pt']),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ankith',
    maintainer_email='ankith@todo.todo',
    description='YOLO Crater Detection',
    license='TODO: License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detector_node = yolo_crater_detector.detector_node:main'
        ],
    },
)

