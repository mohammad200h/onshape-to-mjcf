import setuptools



setuptools.setup(
    name="onshape-to-mjcf",
    version="0.3.26",
    author="Rhoban team",
    author_email="team@rhoban.com",
    description="Converting OnShape assembly to robot definition (MJCF) through OnShape API ",
    packages=setuptools.find_packages(),
    entry_points={
        "console_scripts": [
            "onshape-to-mjcf=onshape_to_mjcf:onshape_to_mjcf.main",

        ]
    },
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    keywords="robot robotics cad design onshape bullet pybullet sdf urdf gazebo ros model kinematics",
    install_requires=[
        "numpy", "requests", "commentjson", "colorama>=0.4.6", "numpy-stl",
         "transforms3d","PrettyPrintTree",
         "mujoco"
    ],
    python_requires='>=3.6',
)