#!/usr/bin/env python
"""qpOASES python distutils setup script."""

#
#  This file is part of qpOASES.
#
#  qpOASES -- An Implementation of the Online Active Set Strategy.
#  Copyright (C) 2007-2015 by Hans Joachim Ferreau, Andreas Potschka,
#  Christian Kirches et al. All rights reserved.
#
#  qpOASES is free software; you can redistribute it and/or
#  modify it under the terms of the GNU Lesser General Public
#  License as published by the Free Software Foundation; either
#  version 2.1 of the License, or (at your option) any later version.
#
#  qpOASES is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
#  See the GNU Lesser General Public License for more details.
#
#  You should have received a copy of the GNU Lesser General Public
#  License along with qpOASES; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
#

#
#   Filename:  setup.py
#   Author:    Sebastian F. Walter, Manuel Kudruss
#   Version:   3.2
#   Date:      2013-2015
#

import os
import numpy as np

from distutils.core import setup
from distutils.extension import Extension
from Cython.Distutils import build_ext
from Cython.Build import cythonize

BASEDIR = os.path.dirname(os.path.abspath(__file__))
BASEDIR = os.path.dirname(BASEDIR)
BASEDIR = os.path.dirname(BASEDIR)

extra_params = {}
extra_params['include_dirs'] = [
    '/usr/include',
    os.path.join(BASEDIR, 'include'),
    os.path.join(BASEDIR, 'include', 'qpOASES'),
    np.get_include()]
extra_params['extra_compile_args'] = ["-O2", "-Wno-unused-variable"]
extra_params['extra_link_args'] = ["-Wl,-O1", "-Wl,--as-needed"]

extra_params = extra_params.copy()
extra_params['libraries'] = ['qpOASES']

extra_params['library_dirs'] = ['/usr/lib', os.path.join(BASEDIR, 'bin')]
extra_params['language'] = 'c++'

if os.name == 'posix':
    extra_params['runtime_library_dirs'] = extra_params['library_dirs']

ext_modules = [
    Extension("qpoases",  ["qpoases.pyx", "qpoases.pxd"],   **extra_params),
]

setup(
    name='qpOASES interface',
    cmdclass={'build_ext': build_ext},
    ext_modules=cythonize(ext_modules),
)
