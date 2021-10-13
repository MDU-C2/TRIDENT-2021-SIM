#!/usr/bin/env python

import roslib
import rospy
import sys,os
from roslaunch import substitution_args
from lxml import etree
import xacro
import resource_retriever
import subprocess
import rospkg
import re
from pathlib import Path

def uwsim_to_abspath(filename, datapath):
    '''
    Returns absolute file path if the file is in the uwsim data path
    '''
    for p in datapath:
        if os.path.lexists(p + '/' + filename):
            return p + '/' + filename, p
    return filename, ''
    
def abspath_to_uwsim(filename, datapath):
    '''
    Returns relative file path
    '''
    expression = re.compile("package://")
    if (expression.match(filename) is None):
	print "fail: ", filename
    else:
	print "found: ", filename

	lista = re.findall("package://[^/]*/", filename)
	print "lista: ", lista, " len: ",len(lista)
	if (len(lista) > 0):
		nomeCompleto = lista[0]
		packageName = nomeCompleto[10:len(nomeCompleto)-1]
		print "packageName: ",packageName
		rospack = rospkg.RosPack()
		print "path: ",rospack.get_path(packageName)
		print "file: ",filename[len(lista[0]):]
		return filename[len(lista[0]):], rospack.get_path(packageName)
   # if ("package://" in filename):
#	rospack = rospkg.RosPack()
#	rospack.list_pkgs() 
#	print "Estimated: ", filename[10:]
#	return rospack.get_path(filename[10:]), 

    for p in datapath:
        if p in filename:
	    if ("file://" in filename) and not ("file://" in p):
		return filename[len(p)+1+len("file://"):], p
	    else:
            	return filename[len(p)+1:], p
    return filename, ''
    
def abspath_to_roslaunch(filename):
    '''
    Tells whether the filename lies in rospath, $home, or elsewhere
    '''
    filetree = filename.split('/')
    # test for ros paths
    for d in range(len(filetree)-1,-1,-1):
        package_path = ''
        try:
            package_path = substitution_args.resolve_args('$(find %s)' % filetree[d])
        except:
            pass
        if package_path != '':
            if package_path == '/'.join(filetree[:d+1]):
                 return '$(find %s)/%s' % (filetree[d], '/'.join(filetree[d+1:]))

    # test for home dir
    userpath = os.path.expanduser('~')
    if userpath in filename:
        return '$(env HOME)/%s' % filename[len(userpath)+1:]
    # other
    return filename
    
def parse_launch_file(launch_file):
    '''
    args: launchfile
    output: uwsim dataPath and scene file
    '''
    launch_xml = etree.parse(launch_file)
    uwsim_node = [node for sub in ['', 'group/'] for node in launch_xml.findall(sub+'node') if node.get('type') == 'uwsim']
    # check node existence
    if len(uwsim_node) == 0:
        rospy.loginfo('No uwsim node found')
        sys.exit(0)
    elif len(uwsim_node) > 2:
        rospy.loginfo('Found several uwsim node, check launchfile')
        sys.exit(1)
        
    # create mapping between uwsim dataPath and absolute/ros paths
    datapath = [roslib.packages.get_pkg_dir('uwsim')+'/data']
    local_datapath = os.path.expanduser('~/.uwsim/data')
    if os.path.lexists(local_datapath):       
        datapath.append(local_datapath)  
        
    # now we have the uwsim node, let's substitute and parse its args
    # we cannot substitute $(arg XXX) outside the launchfile, so we just remove it
    uwsim_args = substitution_args.resolve_args(uwsim_node[0].get('args').replace('$(arg', '')).split(' ')
    print "uwsim_args: ",uwsim_args
    
    # look for configfile and additional datapath in uwsim args
    configfile = ''
    for i in xrange(len(uwsim_args)):
        if uwsim_args[i] == '--configfile':
            i+= 1
            configfile = uwsim_args[i]
        elif uwsim_args[i] == '--dataPath':
            i+= 1
            datapath.append(uwsim_args[i])
            
    # check scene file
    if configfile == '':
        rospy.loginfo('No scene file found in launchfile')
        sys.exit(0)
    configfile, datadir = uwsim_to_abspath(configfile, datapath)
    
    if datadir == '':
        rospy.loginfo('Scene file ' + configfile + ' not found in uwsim dataPath')
  
    return datapath, configfile
    
def extract_xml_value(node, tree):
    '''
    Look for the value nested inside node, according to the given child tree
    '''
    if type(tree) != tuple:
        tree = (tree,)
    parent = v.cloneNode
    value = ''
    for name in tree:
        child = [child for child in parent.childNodes if child.nodeName == name]
        if len(child) == 0:
            rospy.loginfo('Could not find ' + name + ' while exploring ' + v.nodeName)
            sys.exit(1)
        parent = child[0].cloneNode
        
def insert_header_and_write(xml_data, from_file, filename):
    print "calling insert_header_and_write for file "+filename
    '''
    Insert warning at the beginning of the xml structure and write to filename
    '''
    header = ['<!-- %s -->' % c for c in
               [' %s ' % ('=' * 90), 
                ' |    EDITING THIS FILE BY HAND IS NOT RECOMMENDED             %-26s | ' % '', 
                ' |    This document was autogenerated by scene_to_spawner from %-26s | ' % from_file.split('/')[-1], 
                ' %s ' % ('=' * 90)]]
    xml_string = etree.tostring(xml_data, pretty_print=True)
    for comment in header + ['<?xml version="1.0"?>']:
        xml_string = '%s\n%s' % (comment, xml_string)
    with open(filename, 'w') as f:
        f.write(xml_string)
    rospy.loginfo('Creating %s from %s' % (filename, from_file.split('/')[-1]))
    
    
def create_spawner(node, gazebo_model_file):
    '''
    Parse node to know how to spawn the gazebo file
    Returns xml node
    '''
    # spawner node args depend on gazebo model file
    spawn_args = [gazebo_model_file.split('.')[-1] == 'sdf' and 'sdf' or 'urdf']
    # create gazebo spawner group
    name = node.find('name').text
    group_node = etree.Element('group')
    # robot_description if vehicle
    if node.tag == 'vehicle':
        param_node = etree.SubElement(group_node, 'param', name = '%s/robot_description' % name)
        spawn_args.append('%s -param %s/robot_description' % (name, name))
    else:
        param_node = etree.SubElement(group_node, 'param', name = '%s_description' % name)
        spawn_args.append('%s -param %s_description' % (name, name))
    
    param_node.set('command', '$(find xacro)/xacro --inorder %s' % abspath_to_roslaunch(gazebo_model_file))

    # spawner node
    spawn_node = etree.SubElement(group_node, 'node')
    spawn_node.set('name', '%s_spawner' % name)
    spawn_node.set('pkg', 'gazebo_ros')
    spawn_node.set('type', 'spawn_model')
    spawn_node.set('respawn', 'false')
    spawn_node.set('output', 'screen')
    
    # other node args: initial position
    for p in 'xyz':
        spawn_args.append(node.findtext('position/%s' % p).replace(' ',''))
    for p in 'rpy':
        spawn_args.append(node.findtext('orientation/%s' % p).replace(' ',''))
    spawn_node.set('args', '-%s -model %s -x %s -y %s -z %s -R %s -P %s -Y %s' % tuple(spawn_args))
    return group_node
       
       
def process_vehicles(vehicles, datapath):
    '''
    Looks for the vehicle file in datapath
    Resolves corresponding urdf or xacro 
        - convention: uwsim urdf files come from Gazebo urdf files
        - XXX.urdf or XXX.xacro -> XXX_uwsim.urdf
    Creates Gazebo spawner with namespace
    '''
    vehicle_nodes = []
    for vehicle in vehicles:
        name = vehicle.findtext('name')
        uwsim_urdf_file = vehicle.findtext('file')
	print "---> vehicle: " + name + " file: " + uwsim_urdf_file
        # get to Gazebo xacro/urdf file
        gazebo_model_file, datadir = uwsim_to_abspath(uwsim_urdf_file.replace('_uwsim.urdf','.xacro').replace("urdf/","xacro/"), datapath)
        if datadir == '':
            gazebo_model_file, datadir = uwsim_to_abspath(uwsim_urdf_file.replace('_uwsim',''), datapath)            
        if datadir == '':
            rospy.loginfo('Could not find original file for ' + uwsim_urdf_file + ' in ' + scene_file)
            sys.exit(1)
        uwsim_urdf_file = gazebo_model_file.replace('.urdf', '_uwsim.urdf').replace('.xacro', '_uwsim.urdf').replace("xacro/", "urdf/")
            
        print "gazebo_model_file: "+gazebo_model_file
        # parse Gazebo file (urdf or xacro)
        if 'xacro' in gazebo_model_file:
	    print "running xacro"
            uwsim_urdf_xml = subprocess.Popen(['rosrun', 'xacro', 'xacro', '--inorder', gazebo_model_file], stdout=subprocess.PIPE)
            uwsim_urdf_xml = etree.fromstring(uwsim_urdf_xml.stdout.read())
         #   uwsim_urdf_xml = xacro.parse(gazebo_model_file)
         #   xacro.eval_self_contained(uwsim_urdf_xml)
         #   uwsim_urdf_xml = etree.fromstring(uwsim_urdf_xml.toxml())
        else:
            uwsim_urdf_xml = etree.parse(gazebo_model_file).getroot()
	    print "parsing model file"

        # clean up URDF: keep only joints and links
        for child in uwsim_urdf_xml.getchildren():
            if child.tag not in ('joint', 'link'):
                uwsim_urdf_xml.remove(child)
        # and remove collision / inertial / buoyancy from links
        to_be_removed = ('collision', 'inertial', 'buoyancy')
        for link in uwsim_urdf_xml.findall('link'):
            for tag in to_be_removed:
                for node in link.findall(tag):
                    link.remove(node)
        # change mesh url-filenames to uwsim relative filenames
        meshes = uwsim_urdf_xml.findall('link/visual/geometry/mesh')
        if len(meshes) != 0:
            # create uwsim urdf only if mesh in gazebo urdf, otherwise trust the user 
            for mesh in meshes:
		print "------ mesh: "+ mesh.get('filename')
		print "------ " + substitution_args.resolve_args(mesh.get('filename'))
                #mesh_file = resource_retriever.get(substitution_args.resolve_args(mesh.get('filename'))).url[7:]
		mesh_file = substitution_args.resolve_args(mesh.get('filename'))
		#print mesh_file
		print "datapath: ",datapath;

		#mesh_file = substitution_args.resolve_args(mesh.get('filename'))
                # get uwsim relative path for mesh
                mesh_file, datadir = abspath_to_uwsim(mesh_file, datapath)
                #mesh_file, datadir = uwsim_to_abspath(mesh_file, datapath)
		print "======= novooo: "
		print mesh_file
                # if mesh in dae, try to find corresponding osg
                if '.dae' in mesh_file:
                    mesh_osg, datadir_osg = uwsim_to_abspath(mesh_file.replace('.dae','.obj'), datapath)
                    if datadir_osg != '':
                        mesh_file, datadir = abspath_to_uwsim(mesh_osg, datapath)
                if '.stl' in mesh_file:
                    mesh_osg, datadir_osg = uwsim_to_abspath(mesh_file.replace('.stl','.obj'), datapath)
                    if datadir_osg != '':
                        mesh_file, datadir = abspath_to_uwsim(mesh_osg, datapath)
                if datadir == '':
                    rospy.loginfo('Could not find relative path for ' + mesh_file + ' in ' + gazebo_model_file)
                    sys.exit(1)
                else:
                    if os.path.lexists(datadir + '/' + os.path.splitext(mesh_file)[0] + '.osg'):
                        mesh_file = mesh_file.split('.')[0] + '.osg'
                    mesh.set('filename', mesh_file)
            # write uwsim urdf
            insert_header_and_write(uwsim_urdf_xml, gazebo_model_file, uwsim_urdf_file)
        # get nodes to write in gazebo spawner
        vehicle_nodes.append(create_spawner(vehicle, gazebo_model_file))
              
    return vehicle_nodes
    
def process_objects(objcts, datapath):
    '''
    Looks for the objects' mesh files in uwsim datapath
    Creates corresponding urdf for Gazebo
    Creates Gazebo spawner for these objects
    '''
    
    gazebo_mesh_ext = ['.stl', '.dae']
    object_nodes = []
    for obj in scene_xml.findall('object'):
        name = obj.findtext('name')
        uwsim_mesh_file = obj.findtext('file')
        
        # look for compatible gazebo mesh file
        for ext in gazebo_mesh_ext:
            gazebo_mesh_file, datadir = uwsim_to_abspath(os.path.splitext(uwsim_mesh_file)[0] + ext, datapath)
            if datadir != '':
                break
        
        # get object name and gazebo model file name
        gazebo_model_file = os.path.splitext(gazebo_mesh_file)[0] + '.sdf'
        
        # create sdf xml
        object_sdf = etree.Element('sdf', version= '1.4')
        object_model = etree.SubElement(object_sdf, 'model', name=name)
        object_static = etree.SubElement(object_model, 'static')
        object_static.text = 'true'
        object_link = etree.SubElement(object_model, 'link', name= 'link')
        for node in ('visual', 'collision'):
            object_node = etree.SubElement(object_link, node, name=node)
            object_geom = etree.SubElement(object_node, 'geometry')
            object_mesh = etree.SubElement(object_geom, 'mesh')
            object_uri = etree.SubElement(object_mesh, 'uri')
            object_uri.text = 'file://' + abspath_to_roslaunch(gazebo_mesh_file)
            if node == 'collision':
                object_surf = etree.SubElement(object_node, 'surface')
                object_friction = etree.SubElement(object_surf, 'friction')
                object_ode = etree.SubElement(object_friction, 'ode')
                mus = {'mu': '100', 'mu2': '50'}
                for mu in mus:
                    object_mu = etree.SubElement(object_ode, mu)
                    object_mu.text = mus[mu]
        insert_header_and_write(object_sdf, gazebo_mesh_file, gazebo_model_file)
        
        # build gazebo spawner
        object_nodes.append(create_spawner(obj, gazebo_model_file))  
        
    return object_nodes
    
if __name__ == '__main__':
    
    rospy.init_node('scene_to_spawner')
    
    # parse launch file to get uwsim info
    launch_file = sys.argv[1]
    print "Parsing launch file: ", launch_file
    for i in range(0, len(sys.argv)):
	print "Argument[",i,"]: ",sys.argv[i]
    datapath, scene_file = parse_launch_file(launch_file)
    print "----------------- Scene file"
    print "----------------- Scene file ", scene_file;
    print "----------------- datapath ", datapath;

    
    # parse scene file 
    scene_xml = etree.parse(scene_file)
    # get vehicles (ie objects that can be moved through topics)
    vehicle_nodes = process_vehicles(scene_xml.findall('vehicle'), datapath)
    # get objects
    object_nodes = process_objects(scene_xml.findall('object'), datapath)
        
    # create spawner file
    spawner_file = launch_file.replace('.launch', '_spawner.launch')

    spawner_xml = etree.Element('launch')
    for vehicle_node in vehicle_nodes:
        spawner_xml.append(vehicle_node)
    for object_node in object_nodes:
        spawner_xml.append(object_node)
    insert_header_and_write(spawner_xml, launch_file, spawner_file)
    print "----Finished!"
    