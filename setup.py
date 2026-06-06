from setuptools import find_packages, setup

package_name = 'zx200_autonomy'

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
    maintainer='Ryoya SATO',
    maintainer_email='satoryoya1012711@gmail.com',
    description='Sample Program for Opera-sim(PhysX)',
    license='Apach-2.0-License',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'state_machine_node = zx200_autonomy.state_machine_node:main'
        ],
    },
)
