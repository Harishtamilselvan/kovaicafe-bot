from setuptools import find_packages, setup

package_name = 'ordermng'

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
    maintainer='HARISH T',
    maintainer_email='harishtamilselvan864@gmail.com',
    description='TODO: kovaicafe_bot order management package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
          'butler_nav = ordermng.butler_nav:main'
        ],
    },
)
