#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2015, JSK Robotics Laboratory.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of JSK Robotics Laboratory. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

import sys
import argparse

from geneus.geneus_main import package_depends
# we need this for geneus <= 2.2.2 (hydro release)
import geneus.geneus_main
if geneus.geneus_main.pkg_map is None:
    geneus.geneus_main.pkg_map = geneus.geneus_main.get_pkg_map()
# we need this for geneus <= 2.2.4
def get_pkg_map():
    from catkin_pkg import packages, workspaces
    pkg_map = {}
    for ws in workspaces.get_spaces():
        pkgs = packages.find_packages(ws)
        for pkg in pkgs.values():
            if not pkg_map.has_key(pkg.name):
                pkg_map[pkg.name] = pkg
    return pkg_map
geneus.geneus_main.pkg_map = get_pkg_map()


def _get_output(package):
    depends = package_depends(package)
    depends = ['"{0}"'.format(d) for d in depends]
    output = [
        r'set({pkg}_ALL_RUN_DEPENDS {depends})'.format(
            pkg=package, depends=' '.join(depends))
        ]
    return output


def main():
    parser = argparse.ArgumentParser(
        description="Extract all dependencies to output file.")
    parser.add_argument('package')
    parser.add_argument('outfile')
    args = parser.parse_args(sys.argv[1:])

    lines = _get_output(package=args.package)
    with open(args.outfile, 'w') as ofile:
        ofile.write('\n'.join(lines))


if __name__ == '__main__':
    main()
