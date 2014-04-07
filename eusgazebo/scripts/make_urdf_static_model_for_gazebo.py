#! /usr/bin/env python

import sys
import os
import commands


def make_static_model (name, urdf_path = (os.getenv("HOME") + "/.gazebo/models"), overwrite=True):

    urdf_dir_path = urdf_path + "/" + name
    static_urdf_dir_path = urdf_dir_path + '_static'
    urdf_path = urdf_dir_path + '/' + 'model.urdf'
    static_urdf_path = static_urdf_dir_path + '/' + 'model.urdf'

    if overwrite:
        os.system("rm -rf %s" % static_urdf_dir_path)
    else:
        if os.path.exists(static_urdf_dir_path):
            print '[ERROR] the same name static model already exits'
            exit(1)
    
    os.system("cp -r %s %s" % (urdf_dir_path, static_urdf_dir_path))

    if len(commands.getoutput("grep \"<static>false</static>\" %s" % static_urdf_path)) != 0:
        os.system('sed -i -e \"s@<static>false</static>@<static>true</static>@g\" %s' % static_urdf_path)
    elif len(commands.getoutput("grep \"<static>true</static>\" %s" % static_urdf_path)) == 0:
        os.system('sed -i -e \"s@  <link @  <gazebo>\\n    <static>true</static>\\n  </gazebo>\\n  <link @g\" %s' % static_urdf_path)
        # os.system('sed -i -e \"s@</robot>@  <gazebo>\\n    <static>true</static>\\n  </gazebo>\\n</robot>@g\" %s' % static_urdf_path)

    os.system('sed -i -e "s@<robot name=\\"\(.*\)\\"@<robot name=\\"\\1_static\\"@g\" %s' % static_urdf_path)


if __name__ == '__main__':
    if len(sys.argv) > 2:
        make_static_model(sys.argv[1], sys.argv[2])
    elif len(sys.argv) > 1:
        make_static_model(sys.argv[1])
