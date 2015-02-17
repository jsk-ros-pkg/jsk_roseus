#!/usr/bin/env python
# -*- coding:utf-8 -*-

import os
import sys
from catkin_pkg.packages import find_packages
from catkin_pkg.topological_order import topological_order


def main(pkg_name=sys.argv[1]):
    base_path_str = os.getenv('CMAKE_PREFIX_PATH', None)
    if base_path_str is None:
        print "Invalid CMAKE_PREFIX_PATH!"
    base_paths = base_path_str.replace('/devel','').replace('/install','').split(':')
    for base_path in base_paths:
        res = topological_order(base_paths[0], pkg_name)
        if res:
            pkg = res[0][1]
            print os.linesep.join([dep.name for dep in pkg.build_depends])
            return



if __name__ == '__main__':
    main()
