#!/usr/bin/env python

'''
setup.py - Python distutils setup file for BreezySLAM package.

Copyright (C) 2014 Simon D. Levy

This code is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as 
published by the Free Software Foundation, either version 3 of the 
License, or (at your option) any later version.

This code is distributed in the hope that it will be useful,     
but WITHOUT ANY WARRANTY without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU Lesser General Public License 
along with this code.  If not, see <http://www.gnu.org/licenses/>.
'''

# Support streaming SIMD extensions

from platform import machine

OPT_FLAGS  = []
SIMD_FLAGS = []

arch = machine()

print(arch)

if  arch in ['i686', 'x86_64']:
    SIMD_FLAGS = ['-msse3']
    arch = 'i686'

elif arch == 'armv7l':
    OPT_FLAGS = ['-O3']
    SIMD_FLAGS = ['-mfpu=neon']

else:
    arch = 'sisd'

SOURCES = [
    'pybreezyslam.c', 
    'pyextension_utils.c', 
    '../c/coreslam.c', 
    '../c/coreslam_' + arch + '.c',
    '../c/random.c',
    '../c/ziggurat.c']

from distutils.core import setup, Extension

module = Extension('pybreezyslam', 
    sources = SOURCES, 
    extra_compile_args = ['-std=gnu99'] + SIMD_FLAGS + OPT_FLAGS
    )


setup (name = 'BreezySLAM',
    version = '0.1',
    description = 'Simple, efficient SLAM in Python',
    packages = ['breezyslam'],
    ext_modules = [module],
    author='Simon D. Levy and Suraj Bajracharya',
    author_email='simon.d.levy@gmail.com',
    url='https://github.com/simondlevy/BreezySLAM',
    license='LGPL',
    platforms='Linux; Windows; OS X',
    long_description = 'Provides core classes Position, Map, Laser, Scan, and algorithm CoreSLAM'
    )
