from setuptools import find_packages, setup

package_name = 'py_examples'

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
    maintainer='harshwadibhasme',
    maintainer_email='harsh.wadibhasme@addverb.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "demo_cartesian_impedance=py_examples.demo_cartesian_impedance:main",
            "demo_cartesian_jogging=py_examples.demo_cartesian_jogging:main",
            "demo_effort=py_examples.demo_effort:main",
            "demo_gravity_comp_effort=py_examples.demo_gravity_comp_effort:main",
            "demo_gripper=py_examples.demo_gripper:main",
            "demo_joint_impedance=py_examples.demo_joint_impedance:main",
            "demo_joint_jogging=py_examples.demo_joint_jogging:main",
            "demo_ptp_joint=py_examples.demo_ptp_joint:main",
            "demo_ptp_tcp=py_examples.demo_ptp_tcp:main",
            "demo_velocity=py_examples.demo_velocity:main",
        ],
    },
)

