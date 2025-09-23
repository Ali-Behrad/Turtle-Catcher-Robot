from setuptools import find_packages, setup

package_name = 'turtle_catcher_bot'

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
    maintainer='t1kto0rx',
    maintainer_email='t1kto0rx@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "turtle_spawner = turtle_catcher_bot.turtle_spawner:main",
            "turtle_controller = turtle_catcher_bot.turtle_controller:main"
        ],
    },
)
