import trimesh
import numpy as np
from shapely.geometry import LineString
from shapely.geometry.point import Point
import pyglet
from trimesh import intersections as inter
from trimesh import repair as rep
import networkx as nx
from rtree import Rtree
from trimesh import boolean as bool
import meshpy
import sys
import math

def mesh_bounding_box(mesh):
    xl  = trimesh.bounds.corners(mesh.bounds)[0,0]
    xh = trimesh.bounds.corners(mesh.bounds)[1,0]
    yh = trimesh.bounds.corners(mesh.bounds)[2,1]
    zl = trimesh.bounds.corners(mesh.bounds)[3,2]
    yl = trimesh.bounds.corners(mesh.bounds)[4,1]
    zh = trimesh.bounds.corners(mesh.bounds)[5, 2]
    x_size = xh - xl
    y_size = yh - yl
    z_size = zh - zl
    bounding_box_dictionray={'xl':xl,'xh':xh,'yl':yl,'yh':yh,'zl':zl,'zh':zh,
      'x_size':x_size, 'y_size':y_size,'z_size':z_size}
    return bounding_box_dictionray

def create_list_of_possible_slice_sizes(Optimal_slice_size):
    list1 = [Optimal_slice_size]
    list2 = [i for i in range(Optimal_slice_size//2,Optimal_slice_size)]
    list2.reverse()
    list3 = [i for i in range(Optimal_slice_size + 1, Optimal_slice_size + 1 + Optimal_slice_size//2)]
    Possible_slice_sizes = [Optimal_slice_size] + list2 + list3
    return Possible_slice_sizes

def is_slicing_valid(former_plane_2d,potential_plane_2d,height_differences):
    if(potential_plane_2d.area < 30):
        return False
    if(potential_plane_2d.body_count > 1):
        return False
    #if (former_plane_2d != None):
    #    incline_angle = np.arctan((height_differences*np.sqrt(np.pi))/(np.sqrt(former_plane_2d.area)-np.sqrt(potential_plane_2d.area)))
    #    if (incline_angle < np.pi/3 and incline_angle > 0):
    #        return False

    return True

def is_connector_valid(connector, slice_bottom, slice_top, connector_radius):
    epsilon = 0.01
    copy_connector = connector.copy()
    #first check if the connector is on the top plane of the slice
    z_coordinate = trimesh.bounds.corners(slice_bottom.bounds)[5, 2] - epsilon
    center_of_circle = copy_connector.centroid
    angle_samples = np.linspace(0.0,2*np.pi,50)
    first_cond = True
    for theta in angle_samples:
        x_addition = connector_radius*np.cos(theta)
        y_addition = connector_radius*np.sin(theta)
        x = center_of_circle[0] + x_addition
        y = center_of_circle[1] + y_addition
        point = np.array([x,y,z_coordinate])
        point = np.expand_dims(point,0)
        if(slice_bottom.contains(point) == False):
            return False


    #now check if the connector is inside the top slice
    copy_connector.apply_translation([0,0,-trimesh.bounds.corners(copy_connector.bounds)[3,2] + trimesh.bounds.corners(slice_top.bounds)[3,2] + epsilon])
    second_cond = slice_top.contains(copy_connector.vertices)
    if False in second_cond:
        return False
    return True

def add_bolt_to_slice(bolt,slice,top_plane_height):
    bolt_copy = bolt.copy()
    bolt_copy.apply_translation([0,0,top_plane_height])
    slice = bool.boolean_automatic([slice, bolt_copy], 'union')
    if slice.is_watertight == False:
        return None
    return slice

def add_void_to_slice(bolt,slice,bottom_plane_height):
    bolt_copy = bolt.copy()
    bolt_copy.apply_translation([0,0,bottom_plane_height])
    slice = bool.boolean_automatic([slice, bolt_copy], 'difference')
    if slice.is_watertight == False:
        return None
    return slice