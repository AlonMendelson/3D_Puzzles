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
import utilities
import xy_division
import argparse
import os.path
from os import path


def is_COM_in_bottom_mesh(top_mesh,bottom_mesh):
    top_mesh_com = top_mesh.center_mass
    top_mesh_mass = top_mesh.mass

    epsilon = 0.01
    z_coordinate = bottom_mesh.bounds[1,2] - epsilon
    point = np.array([top_mesh_com[0],top_mesh_com[1],z_coordinate])
    point = np.expand_dims(point,0)
    com_in_bottom_mesh = bottom_mesh.contains(point)

    return (top_mesh_com,top_mesh_mass,com_in_bottom_mesh)

def strengh_test(COM_list_item,X_grid,Y_grid,boolean_grid,K):
    COM = COM_list_item[0]
    mass = COM_list_item[1]
    is_inside = COM_list_item[2]

    K = min(K,np.sum(np.array(boolean_grid)))

    if(is_inside):
        return 0

    COM_distance_from_bolts = []
    for x in range(len(X_grid)):
        for y in range(len(Y_grid)):
            if boolean_grid[y][x]:
                distance = np.sqrt(np.square(COM[0] - X_grid[x]) + np.square(COM[1] - Y_grid[y]))
                COM_distance_from_bolts.append(distance)

    COM_distance_from_bolts.sort()
    moment = 0.0
    total_distance_K = 0.0
    for i in range(K):
        total_distance_K += COM_distance_from_bolts[i]

    distance_weights = np.array(COM_distance_from_bolts)/total_distance_K
    for i in range(K):
        moment += COM_distance_from_bolts[i] * mass * distance_weights[i] #each bolt suffers pressure equals the moment/K

    moment = moment/1000000*9.8 #convert from gr*mm to N*m
    return moment/K #return moment per single bolt

