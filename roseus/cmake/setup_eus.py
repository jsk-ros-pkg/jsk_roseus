#!/usr/bin/env python
# -*- coding: utf-8 -*-

import argparse
import os


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('project_name')
    parser.add_argument('eus_package_dir')
    parser.add_argument('output_dir')
    args = parser.parse_args()

    project_name = args.project_name
    eus_package_dir = args.eus_package_dir
    output_dir = args.output_dir

    if not os.path.exists(output_dir):
        os.mkdir(output_dir)
    setup_dot_l = os.path.join(output_dir, 'setup.l')
    with open(setup_dot_l, 'w') as f:
        f.write('''\
(unless (find-package "{pkg}")
(make-package "{pkg}"))
(In-package "{pkg}")
(setq eusdir "{eusdir}")
    '''.format(pkg=project_name.upper(), eusdir=eus_package_dir))


if __name__ == '__main__':
    main()
