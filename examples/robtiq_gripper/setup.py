
from setuptools import find_packages, setup


setup(
    name="robtiq_gripper",
    packages = find_packages(),
    include_package_data = False,
    python_requires='>=3',
    author="Some Dude or Lady",
    license="MIT",
    install_requires=[
        "mujoco"
    ],
    zip_safe=False
)
