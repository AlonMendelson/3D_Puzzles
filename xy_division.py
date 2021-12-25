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

#class that represents a partition tree
class partition:
    def __init__(self,cut_description):
        self.partition_axis = cut_description['axis']
        self.x = cut_description['x']
        self.y = cut_description['y']
        self.first = None
        self.second = None

    def add_partition(self,partition):
        if partition == None:
            return
        if(self.first == None):
            self.first = partition
        else:
            self.second = partition
    def get_partition_axis(self):
        return self.partition_axis


#finds a bounding box of the connectors grid in a single slice
def find_borders_of_connectores(matrix):
    if(matrix is None):
        return None
    holes_array = np.array(matrix)
    coordinates_of_holes = np.argwhere(holes_array != False)
    y_top_border = coordinates_of_holes.min(axis=0)[0]
    y_bottom_border = coordinates_of_holes.max(axis=0)[0]
    x_left_border = coordinates_of_holes.min(axis=0)[1]
    x_right_border = coordinates_of_holes.max(axis=0)[1]
    return {'y_top':y_top_border, 'y_bottom': y_bottom_border, 'x_left': x_left_border, 'x_right':x_right_border}

#checks if a partiton of a slice is valid in a sense that the previous slice hasnt been cut in the same place
def is_partition_valid(previous_layer_partition_x,previous_layer_partition_y,cut_description):

    if(previous_layer_partition_x  is None):
        return True

    if cut_description['axis'] == 'x':
        relevant_section = previous_layer_partition_x[cut_description['y'][0]:cut_description['y'][1] + 1,cut_description['x'][0]:cut_description['x'][1] + 1]
    else:
        relevant_section = previous_layer_partition_y[cut_description['y'][0]:cut_description['y'][1] + 1,
                           cut_description['x'][0]:cut_description['x'][1] + 1]

    if True in relevant_section:
        return False
    return True

#partitions a slice recursively
def do_partition(voids_array, bolts_array,borders,previous_layer_partition_x,previous_layer_partition_y):

    #for the first layer we only have bolts
    if(voids_array is None):
        bolts_in_slice = np.count_nonzero(bolts_array[borders['y_top']:borders['y_bottom'] + 1, borders['x_left']:borders['x_right'] + 1])
        deciding_number = bolts_in_slice

    #for the last layer we only have voids
    elif(bolts_array is None):
        voids_in_slice = np.count_nonzero(voids_array[borders['y_top']:borders['y_bottom'] + 1, borders['x_left']:borders['x_right'] + 1])
        deciding_number = voids_in_slice

    else:
        bolts_in_slice = np.count_nonzero(
            bolts_array[borders['y_top']:borders['y_bottom'] + 1, borders['x_left']:borders['x_right'] + 1])
        voids_in_slice = np.count_nonzero(
            voids_array[borders['y_top']:borders['y_bottom'] + 1, borders['x_left']:borders['x_right'] + 1])
        deciding_number = voids_in_slice



    #we don't want a piece with no voids or no bolts
    if deciding_number == 0  or deciding_number == 1:
        return False , partition({'axis': None, 'x': None, 'y':None})

    x_length = borders['x_right']-borders['x_left'] + 1
    y_length = borders['y_bottom']-borders['y_top'] + 1

    # if the piece is not too big but is connected in at least 2 places stop slicing
    if deciding_number < 6 and deciding_number > 1:
        return True , partition({'axis': None, 'x': None, 'y':None})


    #find the longer dimension
    if x_length > y_length:
        axis = 'x'
    else:
        axis = 'y'


    #try to slice first along the longer dimension
    if(axis == 'x'):
        lower_bound = borders['x_left']
        higher_bound  = borders['x_right']
        middle = (lower_bound + higher_bound)//2
        possible_slicing = [middle]
        for i in range(1,(higher_bound-lower_bound)//2 + 1):
            if middle - i >= lower_bound:
                possible_slicing.append(middle - i)
            if middle + i < higher_bound:
                possible_slicing.append(middle + i)

        for ps in possible_slicing:
            cut_description = {'axis': 'x', 'x': (ps,ps), 'y':(borders['y_top'],borders['y_bottom'])}
            if is_partition_valid(previous_layer_partition_x,previous_layer_partition_y,cut_description) == False:
                continue
            new_borders = {'x_left': borders['x_left'], 'x_right':ps,'y_top':borders['y_top'], 'y_bottom':borders['y_bottom']}
            success1, first_tree = do_partition(voids_array,bolts_array,new_borders,previous_layer_partition_x,previous_layer_partition_y)
            if success1 == False:
                continue
            new_borders = {'x_left': ps + 1, 'x_right':borders['x_right'],'y_top':borders['y_top'], 'y_bottom':borders['y_bottom']}
            success2, second_tree = do_partition(voids_array,bolts_array,new_borders,previous_layer_partition_x,previous_layer_partition_y)
            if success1 == True and success2 == True:
                new_node = partition(cut_description)
                new_node.add_partition(first_tree)
                new_node.add_partition(second_tree)
                return True, new_node

        lower_bound = borders['y_top']
        higher_bound  = borders['y_bottom']
        middle = (lower_bound + higher_bound)//2
        possible_slicing = [middle]
        for i in range(1,(higher_bound-lower_bound)//2 + 1):
            if middle - i >= lower_bound:
                possible_slicing.append(middle - i)
            if middle + i < higher_bound:
                possible_slicing.append(middle + i)

        for ps in possible_slicing:
            cut_description = {'axis': 'y', 'x': (borders['x_left'],borders['x_right']), 'y':(ps,ps)}
            if is_partition_valid(previous_layer_partition_x,previous_layer_partition_y,cut_description) == False:
                continue
            new_borders = {'x_left': borders['x_left'], 'x_right':borders['x_right'],'y_top':borders['y_top'], 'y_bottom':ps}
            success1, first_tree = do_partition(voids_array,bolts_array,new_borders,previous_layer_partition_x,previous_layer_partition_y)
            if success1 == False:
                continue
            new_borders = {'x_left': borders['x_left'], 'x_right':borders['x_right'],'y_top':ps + 1, 'y_bottom':borders['y_bottom']}
            success2, second_tree = do_partition(voids_array,bolts_array,new_borders,previous_layer_partition_x,previous_layer_partition_y)
            if success1 == True and success2 == True:
                new_node = partition(cut_description)
                new_node.add_partition(first_tree)
                new_node.add_partition(second_tree)
                return True, new_node

    if(axis == 'y'):
        lower_bound = borders['y_top']
        higher_bound  = borders['y_bottom']
        middle = (lower_bound + higher_bound)//2
        possible_slicing = [middle]
        for i in range(1,(higher_bound-lower_bound)//2 + 1):
            if middle - i >= lower_bound:
                possible_slicing.append(middle - i)
            if middle + i < higher_bound:
                possible_slicing.append(middle + i)

        for ps in possible_slicing:
            cut_description = {'axis': 'y', 'x': (borders['x_left'],borders['x_right']), 'y':(ps,ps)}
            if is_partition_valid(previous_layer_partition_x,previous_layer_partition_y,cut_description) == False:
                continue
            new_borders = {'x_left': borders['x_left'], 'x_right':borders['x_right'],'y_top':borders['y_top'], 'y_bottom':ps}
            success1, first_tree = do_partition(voids_array,bolts_array,new_borders,previous_layer_partition_x,previous_layer_partition_y)
            if success1 == False:
                continue
            new_borders = {'x_left': borders['x_left'], 'x_right':borders['x_right'],'y_top':ps + 1, 'y_bottom':borders['y_bottom']}
            success2, second_tree = do_partition(voids_array,bolts_array,new_borders,previous_layer_partition_x,previous_layer_partition_y)
            if success1 == True and success2 == True:
                new_node = partition(cut_description)
                new_node.add_partition(first_tree)
                new_node.add_partition(second_tree)
                return True, new_node

        lower_bound = borders['x_left']
        higher_bound  = borders['x_right']
        middle = (lower_bound + higher_bound)//2
        possible_slicing = [middle]
        for i in range(1,(higher_bound-lower_bound)//2 + 1):
            if middle - i >= lower_bound:
                possible_slicing.append(middle - i)
            if middle + i < higher_bound:
                possible_slicing.append(middle + i)

        for ps in possible_slicing:
            cut_description = {'axis': 'x', 'x': (ps,ps), 'y':(borders['y_top'],borders['y_bottom'])}
            if is_partition_valid(previous_layer_partition_x,previous_layer_partition_y,cut_description) == False:
                continue
            new_borders = {'x_left': borders['x_left'], 'x_right':ps,'y_top':borders['y_top'], 'y_bottom':borders['y_bottom']}
            success1, first_tree = do_partition(voids_array,bolts_array,new_borders,previous_layer_partition_x,previous_layer_partition_y)
            if success1 == False:
                continue
            new_borders = {'x_left': ps + 1, 'x_right':borders['x_right'],'y_top':borders['y_top'], 'y_bottom':borders['y_bottom']}
            success2, second_tree = do_partition(voids_array,bolts_array,new_borders,previous_layer_partition_x,previous_layer_partition_y)
            if success1 == True and success2 == True:
                new_node = partition(cut_description)
                new_node.add_partition(first_tree)
                new_node.add_partition(second_tree)
                return True, new_node


    ##if slicing along longer dimension give one more chance for a big piece
    #if x_length <= 5 and y_length <= 5:
    #    return True, partition({'axis': None, 'x': None, 'y':None})

    return False, partition({'axis': None, 'x': None, 'y':None})

def compute_layer_partitions_from_tree(partition_tree,layer_partition_x,layer_partition_y):
    if partition_tree.partition_axis is None:
        return layer_partition_x, layer_partition_y

    compute_layer_partitions_from_tree(partition_tree.first,layer_partition_x,layer_partition_y)
    compute_layer_partitions_from_tree(partition_tree.second,layer_partition_x,layer_partition_y)

    if partition_tree.partition_axis == 'x':
        layer_partition_x[partition_tree.y[0]:partition_tree.y[1] + 1,partition_tree.x[0]] = True
    else:
        layer_partition_y[partition_tree.y[0], partition_tree.x[0]:partition_tree.x[1] + 1] = True

    return layer_partition_x, layer_partition_y


def slice_in_xy(voids_matrix,bolts_matrix,previous_layer_partition_x,previous_layer_partition_y):


    #get borders of the voids and bolts arrays
    if(voids_matrix is None):
        voids_borders = None
        voids_array = None
    else:
        voids_array = np.array(voids_matrix)
        voids_borders = find_borders_of_connectores(voids_array)
    if(bolts_matrix is None):
        bolts_array  = None
        bolts_borders = None
    else:
        bolts_array = np.array(bolts_matrix)
        bolts_borders = find_borders_of_connectores(bolts_array)

    #take the smaller borders
    if(voids_borders is None):
        borders = bolts_borders
    elif bolts_borders is None:
        borders = voids_borders
    else:
        borders = {'y_top':min(voids_borders['y_top'],bolts_borders['y_top']),
                   'y_bottom': max(voids_borders['y_bottom'],bolts_borders['y_bottom']),
                   'x_left': min(voids_borders['x_left'],bolts_borders['x_left']),
                   'x_right':max(voids_borders['x_right'],bolts_borders['x_right'])}

    success, partition_tree = do_partition(voids_array, bolts_array,borders,previous_layer_partition_x,previous_layer_partition_y)

    current_layer_partition_x = np.zeros(bolts_array.shape if voids_array is None else voids_array.shape)
    current_layer_partition_y = np.zeros(bolts_array.shape if voids_array is None else voids_array.shape)

    current_layer_partition_x,current_layer_partition_y = compute_layer_partitions_from_tree(partition_tree,current_layer_partition_x,current_layer_partition_y)


    return success, partition_tree,current_layer_partition_x,current_layer_partition_y


def cut(slice,partition_tree,margin,X,Y,slices_list):
    if partition_tree.partition_axis is None:
        slices_list.append(slice)
        return True


    #cut
    if partition_tree.partition_axis == 'y':
        slicing_coordinate = Y[partition_tree.y[0]] - margin
        first = inter.slice_mesh_plane(slice, np.array([0.0, 1.0, 0.0]), np.array([0.0, slicing_coordinate, 0.0]),
                                   cap=True)
        second = inter.slice_mesh_plane(slice, np.array([0.0, -1.0, 0.0]), np.array([0.0, slicing_coordinate, 0.0]),
                                   cap=True)


    if partition_tree.partition_axis == 'x':
        slicing_coordinate = X[partition_tree.x[0]] + margin
        first = inter.slice_mesh_plane(slice, np.array([-1.0, 0.0, 0.0]), np.array([slicing_coordinate, 0.0, 0.0]),
                                   cap=True)
        second = inter.slice_mesh_plane(slice, np.array([1.0, 0.0, 0.0]), np.array([slicing_coordinate, 0.0, 0.0]),
                                   cap=True)

    if (first.is_watertight == False or second.is_watertight == False):
        if(first.fill_holes() == False or second.fill_holes == False):
            return False

    success1 = cut(first,partition_tree.first,margin,X,Y,slices_list)
    success2 = cut(second,partition_tree.second,margin,X,Y,slices_list)

    if success1 and success2:
        return True
    else:
        return False


