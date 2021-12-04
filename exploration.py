import trimesh
import numpy as np
from shapely.geometry import LineString
import pyglet
from trimesh import intersections as inter
from trimesh import repair as rep
import networkx as nx
from rtree import Rtree
from trimesh import boolean as bool
import meshpy

if __name__ == "__main__":
    # load the mesh from filename
    # file objects are also supported
    mesh = trimesh.creation.cone(1.0,2.0)
    # get a single cross section of the mesh

    new_mesh = inter.slice_mesh_plane(mesh,np.array([0.0, 0.0, -1.0]),mesh.centroid,cap=True)
    new_mesh2 = inter.slice_mesh_plane(mesh,np.array([0.0, 0.0, 1.0]),mesh.centroid,cap=True)
    new_mesh3 = bool.boolean_automatic([new_mesh,new_mesh2],'union')
    cylinder_mesh = trimesh.creation.cylinder(0.2,0.2)
    cone_m_cylinder = bool.boolean_automatic([mesh,cylinder_mesh],'difference')
    cone_m_cylinder.show()

    i = 3