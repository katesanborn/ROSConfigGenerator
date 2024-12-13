#!/usr/bin/env python

import subprocess
import os
import rosnode
from rosgraph.masterapi import Master
import rospkg
import time
from catkin_pkg import workspaces
import json
import roslaunch

def get_all_packages():
    rospack = rospkg.RosPack()

    # Get all ROS packages
    all_packages = rospack.list()

    # Default ROS installation path
    default_ros_path = "/opt/ros/"

    # List to store non-default packages
    non_default_packages = []

    for package in all_packages:
        # Check if the package is not part of the default installation
        package_path = rospack.get_path(package)
        if not package_path.startswith(default_ros_path):
            non_default_packages.append(package)

    return non_default_packages

def find_package_path(package_name):
    """Find the path of a ROS 1 package."""
    try:
        package_path = subprocess.check_output(['rospack', 'find', package_name]).decode().strip()
        return package_path
    except subprocess.CalledProcessError:
        print(f"Package '{package_name}' not found.")
        return None


def topics_from_node_package(nodeName, package):
    nodeType = nodeName
    nodeName , _ = os.path.splitext(nodeName)

    node = roslaunch.core.Node(package=package, node_type=nodeType, name=nodeName)

    try:
        launch = roslaunch.scriptapi.ROSLaunch()
        launch.start()

        process = launch.launch(node)

        time.sleep(2)

        master = Master(rosnode.ID) 
   
        state = master.getSystemState() 

        pubs = sorted([t for t, l in state[0] if "/"+nodeName in l]) 
        subs = sorted([t for t, l in state[1] if "/"+nodeName in l]) 

        if "/rosout" in pubs:
            pubs.remove("/rosout")

        process.stop()
        if len(pubs) <= 0 and len(subs) <= 0:
            return [None, None]

        return [pubs, subs]
    except:
        return [None, None]


def find_nodes_in_package(pkg):
    exe = []

    package_path = find_package_path(pkg)
    for root, dirs, files in os.walk(package_path):
        dirs[:] = [d for d in dirs if not d.startswith('.')]
        for filename in files:

            if filename.startswith('.'):
                continue

            if filename.endswith('.out') or filename.endswith('.bin'):
                continue

            file_path = os.path.join(root, filename)
            if os.access(file_path, os.X_OK):
                exe.append(filename)

    c = workspaces.get_spaces()[0]
    devel_path = os.path.join(c, "lib", pkg)
    for root, dirs, files in os.walk(devel_path):
        for filename in files:
            file_path = os.path.join(root, filename)
            if os.access(file_path, os.X_OK):
                exe.append(filename)


    exe = list(set(exe))

    nodes = []

    for e in exe:
        _, ext = os.path.splitext(e)

        if not (ext == "" or ext == ".py"):
            continue

        pubs, subs = topics_from_node_package(e, pkg)
        nodes.append({"node": e, "publishers": pubs, "subscribers": subs}) 

    return nodes

def find_launch_files_in_package(pkg):
    launch_files = []

    package_path = find_package_path(pkg)
    for root, dirs, files in os.walk(package_path):
        dirs[:] = [d for d in dirs if not d.startswith('.')]
        for filename in files:

            if filename.endswith('.launch'):
                path = os.path.join(root, filename)
                relative_path = os.path.relpath(path, package_path)
                launch_files.append({"launch_file": filename, "relative_path": relative_path})

    return launch_files

def get_catkin_config():
    packages = get_all_packages()
    pkg_out = []

    for pkg in packages:
        nodes = find_nodes_in_package(pkg)
        launch_files = find_launch_files_in_package(pkg)
        pkg_out.append({"package": pkg, "nodes": nodes, "launch_files": launch_files})

    return pkg_out

if __name__ == "__main__":
    with open('/config-data/config.json', 'w', encoding='utf-8') as f:
        json.dump(get_catkin_config(), f, ensure_ascii=False, indent=4)