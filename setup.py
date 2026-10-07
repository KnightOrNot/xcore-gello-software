import setuptools

with open("README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
    name="xcore-gello-software",
    version="0.0.1",
    author="Philipp Wu",
    author_email="philippwu@berkeley.edu",
    description="software for GELLO",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/KnightOrNot/xcore-gello-software",
    packages=setuptools.find_namespace_packages(
        include=["xcore_gello_software", "xcore_gello_software.*"]
    ),
    package_data={"": ["*.xml", "*.urdf", "*.json"]},
    entry_points={
        "console_scripts": [
            "xcore-gello-software=xcore_gello_software.cli:main",
        ]
    },
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
    ],
    python_requires=">=3.8",
    license="MIT",
    install_requires=[
        "numpy",
    ],
)
