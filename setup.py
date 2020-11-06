#!/usr/bin/env python3

from setuptools import setup
from setuptools import find_packages


setup(
    name="rowhammer_tester",
    description="LiteX Row Hammer Tester",
    author="Antmicro",
    python_requires="~=3.6",
    install_requires=[
        "pythondata-misc-tapcfg",
        "sphinx",
        "sphinxcontrib-wavedrom",
    ],
    packages=['rowhammer_tester'],
)
