from setuptools import setup

package_name = 'opencv_tools'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='draxn09',
    maintainer_email='draxn09@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'img_publisher = opencv_tools.image_publisher:main',
        'img_subscriber = opencv_tools.image_subscriber:main',
        'img_filter1 = opencv_tools.filter1:main',
        'img_filter2 = opencv_tools.filter_HPF:main',
        'img_filter3 = opencv_tools.filter_CKF:main',
        'red_filter = opencv_tools.red_detector:main',
        'blue_filter = opencv_tools.blue_detector:main',
        'green_filter = opencv_tools.green_detector:main',
        'per_filter = opencv_tools.person_filter:main',
        ],
    },
)
