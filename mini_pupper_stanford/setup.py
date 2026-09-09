from setuptools import find_packages, setup

package_name = "mini_pupper_stanford"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        (
            "share/" + package_name,
            ["package.xml"],
        ),
        (
            'share/' + package_name + '/systemd',
            [
                "systemd/mayday-stanford-locomotion.service",
            ],
        ),
    ],
    install_requires=[
        "setuptools",
    ],
    zip_safe=True,
    maintainer="Mayday",
    maintainer_email="ubuntu@localhost",
    description=(
        "Guarded ROS 2 cmd_vel adapter for "
        "StanfordQuadruped locomotion."
    ),
    license="MIT",
    scripts=[
        "scripts/mayday_stanford_owner",
    ],
    entry_points={
        "console_scripts": [
            (
                "stanford_cmd_vel = "
                "mini_pupper_stanford.stanford_cmd_vel:main"
            ),
        ],
    },
)
