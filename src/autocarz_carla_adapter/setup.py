from setuptools import setup

package_name = 'autocarz_carla_adapter'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (
            'share/' + package_name + '/launch',
            [
                'launch/autocarz_carla.launch.py',
                'launch/carla_real_topic_adapter.launch.py',
            ],
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jnghn42',
    maintainer_email='jnghn42@todo.todo',
    description='CARLA-to-real-topic adapter for autocarz testing.',
    license='TODO',
    entry_points={
        'console_scripts': [
            'carla_real_topic_adapter = autocarz_carla_adapter.carla_real_topic_adapter:main',
            'fake_erp42_serial = autocarz_carla_adapter.fake_erp42_serial:main',
        ],
    },
)
