#!/usr/bin/env python

import setuptools

with open('requirements.txt', 'r') as f:
    install_requires = [line.strip() for line in f.readlines()]

setuptools.setup(
    name='aristos_cpp',
    version='2.0.0',
    description='A complete Coverage Path Planning solver based on AIPlan4EU Unified Planning library.',
    packages=setuptools.find_packages(),
    install_requires=install_requires,
    python_requires='>=3.10',
    zip_safe=False,
    package_data={'aristos_cpp': ['templates/domain.pddl', 'templates/*.j2']}
)