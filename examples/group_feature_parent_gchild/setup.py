
from setuptools import find_packages, setup


setup(
    name="group_feature_parent_gchild",
    packages = find_packages(),
    include_package_data = True,
    python_requires='>=3',
    author="Some Dude or Lady",
    license="MIT",
    install_requires=[
        "mujoco"
    ],
    package_data={'': ['tree.json','assets/*']},
    zip_safe=False
)
