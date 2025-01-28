#!/usr/bin/env python

import subprocess
import os
import rosnode
from rosgraph.masterapi import Master
import rosgraph.masterapi
import rospkg
import time
from catkin_pkg import workspaces
import json
import roslaunch

def get_all_packages() -> list:
    """Returns all ROS packages that are not default

    Returns:
        list[str]: List of all ROS packages
    """    
    
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

    non_default_packages.sort()

    return non_default_packages

def find_package_path(package_name: str) -> str:
    """Gets the path of a ROS packages from its name

    Args:
        package_name (str): Name of package

    Returns:
        str: Path to package
    """    
    
    try:
        package_path = subprocess.check_output(['rospack', 'find', package_name]).decode().strip()
        return package_path
    except subprocess.CalledProcessError:
        print(f"Package '{package_name}' not found.")
        return None


def topics_from_node_package(node_name: str, package: str) -> list:
    """Finds publishers and subscribers in a given node in a given package

    Args:
        node_name (str): Name of node
        package (str): Node package

    Returns:
        list: List of publishers and subscribers
    """    
    
    node_type = node_name
    node_name , _ = os.path.splitext(node_name)

    node = roslaunch.core.Node(package=package, node_type=node_type, name=node_name)

    try:
        launch = roslaunch.scriptapi.ROSLaunch()
        launch.start()

        process = launch.launch(node)

        time.sleep(2)

        master = Master(rosnode.ID) 
   
        state = master.getSystemState() 

        pubs = sorted([t for t, l in state[0] if "/"+node_name in l]) 
        subs = sorted([t for t, l in state[1] if "/"+node_name in l]) 

        if "/rosout" in pubs:
            pubs.remove("/rosout")

        process.stop()
        if len(pubs) <= 0 and len(subs) <= 0:
            return [None, None]

        return [pubs, subs]
    except:
        return [None, None]


def find_nodes_in_package(pkg: str) -> list:
    """Returns a list of all nodes and their publishers and subscribers in a package

    Args:
        pkg (str): Name of package

    Returns:
        list: List of nodes with publishers and subscribers
    """    
    
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

    nodes.sort(key = lambda x: x["node"])

    return nodes

def get_node_topics(node_name: str) -> tuple:
    """Returns the publishers and subscribers for a given node.

    Args:
        node_name (str): Name of node running

    Returns:
        tuple: (List of publishers, list of subscribers)
    """    
    
    master = rosgraph.masterapi.Master('/roscore')
    state = master.getSystemState()

    publishers = [topic for topic, nodes in state[0] if node_name in nodes and "rosout" not in topic]
    subscribers = [topic for topic, nodes in state[1] if node_name in nodes and "rosout" not in topic]
    
    publishers.sort()
    subscribers.sort()

    return publishers, subscribers

def find_launch_files_in_package(pkg: str) -> list:
    """Returns a list of all launch files in a package and the nodes they launch

    Args:
        pkg (str): Package name

    Returns:
        list: List of all launch files and relative path in package
    """    
    
    launch_files = []

    package_path = find_package_path(pkg)
    for root, dirs, files in os.walk(package_path):
        dirs[:] = [d for d in dirs if not d.startswith('.')]
        for filename in files:

            if filename.endswith('.launch'):
                path = os.path.join(root, filename)
                relative_path = os.path.relpath(path, package_path)
                
                try:
                    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
                    roslaunch.configure_logging(uuid)
                    launch = roslaunch.parent.ROSLaunchParent(uuid, [path])
                    launch.start()

                    time.sleep(2)
                    
                    nodes = rosnode.get_node_names()
                    nodes_topics = []

                    for node in nodes:
                        if "rostopic" in node or "rosout" in node:
                            continue
                        
                        pubs, subs = get_node_topics(node)
                        
                        node_name = node[1:] if node.startswith("/") else node
                        
                        nodes_topics.append({"node": node_name, "publishers": pubs, "subscribers": subs})
                                
                    launch.shutdown()
                    
                    nodes_topics.sort(key = lambda x: x["node"])
                    
                    launch_files.append({"launch_file": filename, "relative_path": relative_path, "nodes": nodes_topics})
                except:
                    launch_files.append({"launch_file": filename, "relative_path": relative_path, "nodes": []})
                

    launch_files.sort(key = lambda x: x["launch_file"])

    return launch_files

def get_catkin_config() -> list:
    """Gets all packages, nodes, and launch files

    Returns:
        list: List of all packages and associated nodes and launch files
    """    
    
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