from setuptools import setup

package_name = 'my_test_pkg_py'

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
    maintainer='yunseo',
    maintainer_email='hwangyunseo326@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
	'master_order = my_test_pkg_py.master_order:main',
    'node1 = my_test_pkg_py.node1:main',
    # 'node2 = my_test_pkg_py.node2:main',
    
]
    },
)
