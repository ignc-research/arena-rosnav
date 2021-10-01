from setuptools import setup
from Cython.Build import cythonize
import numpy


setup(
    name="crings",
    description='crings',
    author='Daniel Dugas',
    version='0.0.14',
    # doesn't work with pip install -e
    ext_modules=cythonize("crings.pyx", annotate=True),
    python_requires='>=3.6, <3.7',
    include_dirs=[numpy.get_include()],
)
