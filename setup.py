from setuptools import find_packages, setup

package_name = 'simulado_af'

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
    maintainer='borg',
    maintainer_email='vinixviii@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'q1 = simulado_af.q1:main',
            'q2 = simulado_af.q2:main',
            'q3 = simulado_af.q3:main',
        ],
    },
)
