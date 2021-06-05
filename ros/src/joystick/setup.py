from setuptools import setup

package_name = 'joystick'

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
    maintainer='Jim',
    maintainer_email='dorajim15@gmail.com',
    description='handle joystick',
    license='',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = joystick.publisher:main',
            'listener = joystick.subscriber:main',
            'serve = joystick.serve:main',
        ],
    },
)
