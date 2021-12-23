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


if __name__ == "__main__":

    #parameters definition
    Optimal_slice_size = 20
    Connector_radius = 4
    Connector_height = 3
    space_between_connectors = 3.2


    
    # load the mesh from models library
    full_body = trimesh.load_mesh('models/Bunny-LowPoly.stl')

    #get a dictionary with sizes of the bounding box of the mesh
    bounding_box_dictionary = utilities.mesh_bounding_box(full_body)

    #get a list with the possible slice sizes ordered by optimality
    Possible_slice_sizes = utilities.create_list_of_possible_slice_sizes(Optimal_slice_size)

    #create a list of slices along the z direction
    slices=[]

    #create a list of cross sections of the slices
    cross_sections = []

    #set height to slice to the first height
    height_of_remaining_body = bounding_box_dictionary['z_size']

    #count the number of slices
    number_of_slices = 0

    former_slice = None

    #create a copy of the mesh
    full_body_2 = full_body.copy(include_cache=False)

    #start slicing in z direction
    while(height_of_remaining_body > Optimal_slice_size):

        #translate the mesh to stand on z=0
        full_body_2.apply_translation([0, 0, -trimesh.bounds.corners(full_body_2.bounds)[0, 2]])

        #try a single slice
        slice_valid = False
        height_difference = Possible_slice_sizes[0]
        number_of_tries = 0
        while (slice_valid == False and height_difference < height_of_remaining_body and number_of_tries < len(Possible_slice_sizes)-1):
            #get cross section of potential slice
            cs_3d = full_body_2.section(np.array([0.0,0.0,1.0]),np.array([0.0,0.0,height_difference]))
            #transfer to 2d
            cs_2d,_ = cs_3d.to_planar()
            #check slicing validity
            slice_valid = utilities.is_slicing_valid(former_slice, cs_2d, height_difference)
            if(slice_valid):
                former_slice = cs_2d
            else:
                number_of_tries += 1
                height_difference = Possible_slice_sizes[number_of_tries]

        #check if a slice has been found
        if slice_valid:
            slice = inter.slice_mesh_plane(full_body_2, np.array([0.0, 0.0, -1.0]), np.array([0.0, 0.0, height_difference]),
                                       cap=True)
            full_body_2 = inter.slice_mesh_plane(full_body_2, np.array([0.0, 0.0, 1.0]),
                                           np.array([0.0, 0.0, height_difference]),
                                           cap=True)
            if(slice.is_watertight == False or full_body_2.is_watertight == False):
                print('Slicing at z failed watertight')
                sys.exit(0)
            slices.append(slice)
            cross_sections.append(former_slice)
            height_of_remaining_body -= height_difference


        else:
            break

    slices.append(full_body_2)

    #the model has been sliced in the z direction


    #determine locations of potential connectors in a x-y grid
    X = list(np.arange(bounding_box_dictionary['xl'] + space_between_connectors,bounding_box_dictionary['xh'] - space_between_connectors,space_between_connectors + 2 * Connector_radius))
    Y = list(np.arange(bounding_box_dictionary['yl'] + space_between_connectors,bounding_box_dictionary['yh'] - space_between_connectors,space_between_connectors + 2 * Connector_radius))
    Y.reverse()


    #build for each location add a bolt to a grid holding all bolts
    connector = trimesh.creation.cylinder(Connector_radius, Connector_height)
    connectors = []
    for y in Y:
        connectors_row = []
        for x in X:
            connector_copy = connector.copy()
            connector_copy.apply_translation([x,y,Connector_height/2])
            connectors_row.append(connector_copy)
        connectors.append(connectors_row)



    #check which connectors are valid for each slice and save data in array
    # TODO: perhaps check that a bit larger connector works for robustness
    # TODO: raise error if slice has no voids/bolts
    connectors_validation_tensor = []
    for s in range(len(slices)-1):
        counter = 0
        connector_2d_array = []
        for connectors_row in connectors:
            connector_1d_array = []
            for connector in connectors_row:
                if(utilities.is_connector_valid(connector,slices[s],slices[s+1],Connector_radius)):
                    connector_1d_array.append(True)
                    counter += 1
                else:
                    connector_1d_array.append(False)
            connector_2d_array.append(connector_1d_array)
        connectors_validation_tensor.append(connector_2d_array)


    #plan a partition of each slice into pieces. the partition is represented by a tree
    partition_trees = []
    success,partition_tree,partition_layer_x,partition_layer_y = xy_division.slice_in_xy(None,connectors_validation_tensor[0],None,None)
    if success == False:
        print('couldnt divide slice')
        sys.exit(0)
    partition_trees.append(partition_tree)

    for s in range(1,len(slices)-1):
        success, partition_tree, partition_layer_x, partition_layer_y = xy_division.slice_in_xy(connectors_validation_tensor[
                                                                                                    s-1],
                                                                                                connectors_validation_tensor[
                                                                                                    s], partition_layer_x, partition_layer_y)
        if success == False:
            print('couldnt divide slice')
            sys.exit(0)
        partition_trees.append(partition_tree)

    success, partition_tree, partition_layer_x, partition_layer_y = xy_division.slice_in_xy(connectors_validation_tensor[
                                                                                                    -1],
                                                                                                None, partition_layer_x, partition_layer_y)
    if success == False:
        print('couldnt divide slice')
        sys.exit(0)
    partition_trees.append(partition_tree)


    #add bolts to slices - TODO: perhaps make the bolts a bit smaller than the voids so they would fit in each other
    for s in range(len(slices)-1):
        slice_top_plane_height = trimesh.bounds.corners(slice.bounds)[5, 2]
        i = 0
        for connectors_row in connectors:
            j = 0
            for connector in connectors_row:
                if connectors_validation_tensor[s][i][j] == True:
                    new_slice = utilities.add_bolt_to_slice(connector,slices[s],slice_top_plane_height)
                    if new_slice == None:
                        print('couldnt add bolt')
                    else:
                        slices[s] = new_slice
                j +=1
            i += 1



        # add voids to slices
    for s in range(len(slices) - 1):
        slice_bottom_plane_height = trimesh.bounds.corners(slice.bounds)[3, 2]
        i = 0
        for connectors_row in connectors:
            j = 0
            for connector in connectors_row:
                if connectors_validation_tensor[s][i][j] == True:
                    new_slice = utilities.add_void_to_slice(connector, slices[s+1], slice_bottom_plane_height)
                    if new_slice == None:
                        print('couldnt add void')
                    else:
                        slices[s+1] = new_slice
                j += 1
            i += 1


    #cut each slice according to the design made earlier
    margin = Connector_radius + space_between_connectors/2
    slices_to_print = []
    for s in range(len(slices)):
        slices_list = []
        success = xy_division.cut(slices[s],partition_trees[s],margin,X,Y,slices_list)
        if success == False:
            print('couldnt slice in x,y')
        else:
            for t in slices_list:
                t.show()
            slices_to_print.append(slices_list)

