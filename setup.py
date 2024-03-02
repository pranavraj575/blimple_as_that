from distutils.core import setup
from setuptools import find_packages

setup(
    name='blimple_as_that',
    version='6.9.0',
    packages=find_packages(),
    install_requires=['numpy',
                      'psutil',
                      'zmq',
                      'cbor',
                     ],
    license='Liscence to Krill',
)
