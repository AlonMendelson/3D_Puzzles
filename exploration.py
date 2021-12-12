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


if __name__ == "__main__":

    Optimal_slice_size = 20
    Connector_grid = 7.5
    Connector_radius = 4
    Connector_height = 3
    space_between_connectors = 3.2
    slice_limmit = 3
    Cube_grid_x = 30
    Cube_grid_y = 15

    
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

    #slices_num = min(slice_limmit, int((bounding_box_dictionary['z_size']) / Optimal_slice_size))
    # We first slice our body on z axis
    #for i in range(0, slices_num):
    #    full_body.apply_translation([0,0,-trimesh.bounds.corners(full_body.bounds)[0,2]])
    #    slice     = inter.slice_mesh_plane(full_body, np.array([0.0, 0.0, -1.0]), np.array([0.0, 0.0, Optimal_slice_size]), cap=True)
    #    full_body = inter.slice_mesh_plane(full_body, np.array([0.0, 0.0,  1.0]), np.array([0.0, 0.0, Optimal_slice_size]), cap=True)
    #    if not slice.is_watertight:
    #        print('Slicing at z failed watertight')
    #        sys.exit(0)
    #    slices.append(slice)


    #build a 2d grid of centers of the connectors
    X = list(np.arange(bounding_box_dictionary['xl'] + space_between_connectors,bounding_box_dictionary['xh'] - space_between_connectors,space_between_connectors + 2 * Connector_radius))
    Y = list(np.arange(bounding_box_dictionary['yl'] + space_between_connectors,bounding_box_dictionary['yh'] - space_between_connectors,space_between_connectors + 2 * Connector_radius))
    Y.reverse()


    #build grid of bolts
    connector = trimesh.creation.cylinder(Connector_radius, Connector_height)
    connectors = []
    for y in Y:
        connectors_row = []
        for x in X:
            connector_copy = connector.copy()
            connector_copy.apply_translation([x,y,Connector_height/2])
            connectors_row.append(connector_copy)
        connectors.append(connectors_row)


    #scene = trimesh.Scene()
    #for row in connectors:
    #    for con in row:
    #        scene.add_geometry(con)
    #scene.add_geometry(slices[4])
    #scene.show()

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
        connectors_validation_tensor.append(connector_2d_array)


    #add bolts to slices
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



    # First part adding voids in each slice on our grid
    slices_conn = []
    for z_idx in range(0, slices_num):
        conn_idx = 0
        slices_conn.append(slices[z_idx].copy())
        for x_idx in range(0, int((bounding_box_dictionary['x_size'])/Connector_grid)):
            for y_idx in range(0, int((bounding_box_dictionary['y_size'])/Connector_grid)):
                temp_conn  = connectors[conn_idx].copy().apply_translation([0, 0, Optimal_slice_size / 2])
                all_inside  = not False in [x == True for x in slices[z_idx].contains(temp_conn.vertices)]
                any_inside  = True in [x for x in slices[z_idx].contains(temp_conn.vertices)]
                if all_inside:
                    #Temp fix till
                    temp = bool.boolean_automatic([slices_conn[z_idx],connectors[conn_idx]],'difference')
                    if temp.is_watertight:
                        slices_conn[z_idx] = temp.copy()
                    if not slices_conn[z_idx].is_watertight:
                        print('Adding voids failed watertight')
                        sys.exit(0)
                conn_idx += 2

    # Second part adding bolts in each slice on our grid
    for z_idx in range(0, slices_num):
        conn_idx = 0
        for x_idx in range(0, int((bounding_box_dictionary['x_size'])/Connector_grid)):
            for y_idx in range(0, int((bounding_box_dictionary['y_size'])/Connector_grid)):
                temp_conn = connectors[conn_idx].copy().apply_translation([0, 0, Optimal_slice_size / 2])
                all_inside = not False in [x == True for x in slices[z_idx].contains(temp_conn.vertices)]
                any_inside = True in [x  for x in slices[z_idx].contains(temp_conn.vertices)]
                if all_inside:
                    slices_conn[z_idx] = trimesh.util.concatenate([slices_conn[z_idx],connectors[conn_idx+1]])
                    if not slices_conn[z_idx].is_watertight:
                        print('Adding bolts failed watertight')
                        sys.exit(0)
                conn_idx += 2

    # Sanity check see cross section of voids and bolts in slice 0
    for z_idx in range(0, slices_num):
        slice = slices_conn[z_idx].section(plane_origin=[0,0,0.2],plane_normal=[0,0,1])
        slice.show()
        slice = slices_conn[z_idx].section(plane_origin=[0, 0, Optimal_slice_size + 0.2], plane_normal=[0, 0, 1])
        slice.show()

    # First slicing over x axis
    slices_x_cut = []
    for z_idx in range(0, slices_num):
        current_slice = slices_conn[z_idx]
        current_x_low  = trimesh.bounds.corners(current_slice.bounds)[0,0]
        current_x_high = trimesh.bounds.corners(current_slice.bounds)[1,0]
        for x_idx in range(0,int((current_x_high-current_x_low)/Cube_grid_x)):
            current_slice.apply_translation([-trimesh.bounds.corners(current_slice.bounds)[0,0], 0, 0])
            slice         = inter.slice_mesh_plane(current_slice,np.array([-1.0, 0.0, 0.0]),np.array([Cube_grid_x, 0.0, 0.0]),cap=True)
            current_slice = inter.slice_mesh_plane(current_slice,np.array([ 1.0, 0.0, 0.0]),np.array([Cube_grid_x, 0.0, 0.0]),cap=True)
            slices_x_cut.append(slice)

    # Second slice over y axis
    slices_y_cut = []
    for x_idx in range(0, len(slices_x_cut)):
        current_slice = slices_x_cut[x_idx]
        current_y_low  = trimesh.bounds.corners(current_slice.bounds)[0,1]
        current_y_high = trimesh.bounds.corners(current_slice.bounds)[2,1]
        for y_idx in range(0,int((current_y_high-current_y_low)/Cube_grid_y)):
            current_slice.apply_translation([0, -trimesh.bounds.corners(current_slice.bounds)[0,1], 0])
            slice         = inter.slice_mesh_plane(current_slice,np.array([0.0, -1.0, 0.0]),np.array([0.0, Cube_grid_y, 0.0]),cap=True)
            current_slice = inter.slice_mesh_plane(current_slice,np.array([0.0,  1.0, 0.0]),np.array([0.0, Cube_grid_y, 0.0]),cap=True)
            slices_y_cut.append(slice)

    i = 3
    #slices_y_cut[3].show(flags={'wireframe': True, 'axis': True})