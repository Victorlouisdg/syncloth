import setuptools
from setuptools import find_packages

setuptools.setup(
    name="syncloth",
    version="0.0.1",
    author="Victor-Louis De Gusseme",
    author_email="victorlouisdg@gmail.com",
    description="TODO",
    install_requires=[
        "numpy",
        "scipy",
        "triangle",
    ],
    packages=find_packages(),
)
