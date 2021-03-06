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
import strengh_testing
import argparse
import os.path
from os import path


def create_puzzle(args):

    #parameters definition
    Optimal_slice_size = 20
    Connector_radius = 4
    Connector_height = 3
    space_between_connectors = 3.2
    Density = 1.4/1000 #for PLA translated to gr/mm^3
    Fill_ratio = 0.1 #infill ratio of 3D printer
    Fill_overhead = 3 #constant after measurements of infill overhead from slicing
    Knn = 3 #how many bolts to take into consideration when testing strength
    Maximum_moment = 1 #N*m according to our experiment

    # load the mesh from models library
    full_body = trimesh.load_mesh('models/' + args.input_model + '.stl')

    full_body.density = Density * Fill_ratio * Fill_overhead

    #get a dictionary with sizes of the bounding box of the mesh
    bounding_box_dictionary = utilities.mesh_bounding_box(full_body)

    #get a list with the possible slice sizes ordered by optimality
    Possible_slice_sizes = utilities.create_list_of_possible_slice_sizes(Optimal_slice_size)

    #create a list of slices along the z direction
    slices=[]

    #center_of_masses_above_slice
    COM_above_slice = []

    #create a list of cross sections of the slices
    cross_sections = []

    #set height to slice to the first height
    height_of_remaining_body = bounding_box_dictionary['z_size']

    #count the number of slices
    number_of_slices = 0

    former_slice = None

    #create a copy of the mesh
    full_body_2 = full_body.copy(include_cache=False)

    full_body_2.density = full_body.density

    #start slicing in z direction
    while(height_of_remaining_body > Optimal_slice_size):

        #translate the mesh to stand on z=0
        full_body_2.apply_translation([0, 0, -full_body_2.bounds[0,2]])

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
            slice_valid = utilities.is_slicing_valid(former_slice, cs_2d, height_difference,Connector_radius,space_between_connectors)
            if(slice_valid):
                former_slice = cs_2d
            else:
                number_of_tries += 1
                height_difference = Possible_slice_sizes[number_of_tries]

        #check if a slice has been found
        if slice_valid:
            slice = inter.slice_mesh_plane(full_body_2, np.array([0.0, 0.0, -1.0]), np.array([0.0, 0.0, height_difference]),
                                       cap=True)
            slice.density = full_body_2.density
            full_body_2 = inter.slice_mesh_plane(full_body_2, np.array([0.0, 0.0, 1.0]),
                                           np.array([0.0, 0.0, height_difference]),
                                           cap=True)
            full_body_2.density = slice.density
            COM_above_slice.append(strengh_testing.is_COM_in_bottom_mesh(full_body_2,slice))
            if(slice.is_watertight == False or full_body_2.is_watertight == False):
                print('Slicing at z failed watertight')
                sys.exit(0)

            slices.append(slice)
            cross_sections.append(former_slice)
            height_of_remaining_body -= height_difference


        else:
            break

    #take care of remainding layer if exists
    if(utilities.mesh_bounding_box(full_body_2)['z_size'] > 10):
        full_body_2.apply_translation([0, 0, -full_body_2.bounds[0, 2]])
        slices.append(full_body_2)
    else:
        slices[-1] = bool.union([slices[-1],full_body_2],'scad')
        if slices[-1].is_watertight == False:
            if slices[-1].fill_holes() == False:
                print('Slicing at z failed watertight')
                sys.exit(0)

    #the model has been sliced in the z direction
    #determine locations of potential connectors in a x-y grid
    X = list(np.arange(bounding_box_dictionary['xl'] + space_between_connectors,bounding_box_dictionary['xh'] - space_between_connectors,space_between_connectors + 2 * Connector_radius))
    Y = list(np.arange(bounding_box_dictionary['yl'] + space_between_connectors,bounding_box_dictionary['yh'] - space_between_connectors,space_between_connectors + 2 * Connector_radius))
    Y.reverse()


    #for each location add a bolt to a grid holding all bolts
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
        if counter == 0:
            print('puzzle creation failed')
            sys.exit(0)
        connectors_validation_tensor.append(connector_2d_array)

    moments = []
    #strength_testing
    for i in range(len(slices)-1):
        moment = strengh_testing.strengh_test(COM_above_slice[i],X,Y,connectors_validation_tensor[i],Knn)
        moments.append((moment,moment>Maximum_moment))

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


    #add bolts to slices
    for s in range(len(slices)-1):
        slice_top_plane_height = slices[s].bounds[1,2]
        i = 0
        for connectors_row in connectors:
            j = 0
            for connector in connectors_row:
                if connectors_validation_tensor[s][i][j] == True:
                    new_slice = utilities.add_bolt_to_slice(connector,slices[s],slice_top_plane_height,Connector_height,Connector_radius)
                    if new_slice == None:
                        print('couldnt add bolt')
                    else:
                        slices[s] = new_slice
                j +=1
            i += 1



        # add voids to slices
    for s in range(len(slices) - 1):
        slice_bottom_plane_height = slices[s].bounds[0,2]
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
            slices_to_print.append(slices_list)

    for slice in range(len(slices_to_print)):
        parts = slices_to_print[slice]
        for part in range(len(parts)):
            partname = args.input_model + '_slice_'+str(slice)+'_part_'+str(part)
            parts[part].export(args.output_dir + "/"+partname+".stl")


    for i in range(len(slices) - 1):
        if(moments[i][1]):
            print('Warning: the moment on slice ' + str(i) + ' is above limit. Consider printing parts for this slice with extended shells/infill' )

    print("puzzle created successfully")

def main():
    args = sys.argv[1:]
    parser = argparse.ArgumentParser(description='Creates a 3D puzzle')
    parser.add_argument('input_model',  type=str,
                                help="name of model")
    parser.add_argument('output_dir', type=str, help="output directory")
    args = parser.parse_args(args)
    model_path = 'models/' + args.input_model + '.stl'
    output_path = args.output_dir
    if path.exists(model_path) == False:
        print('model file doesnt exist')
        sys.exit(1)
    if path.exists(output_path) == False:
        os.mkdir(output_path)
    create_puzzle(args)

if __name__ == "__main__":
    main()