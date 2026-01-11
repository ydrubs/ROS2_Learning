from setuptools import find_packages, setup

package_name = 'navigation_experiments'

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
    maintainer='ydrubs',
    maintainer_email='ydrubins@yahoo.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "navigate_to_point = navigation_experiments.nav2_test:main",
            "tbot3_house_navigation = navigation_experiments.tbot3_house_navigation:main",
        ],
    },
)
