from setuptools import find_packages, setup

package_name = 'feetech_python_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyserial', 'pyyaml'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='brianjblank7@gmail.com',
    description='Shared Python Feetech servo bus driver and calibration routine',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [],
    },
)
