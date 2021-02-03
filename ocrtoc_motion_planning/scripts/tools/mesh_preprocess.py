import trimesh 
import tf
import os
import pathlib
import numpy as np
Re = np.array([[1.,0,0],[0,0,-1],[0,1,0]])
#pyassimp_mesh = pyassimp.load("/root/ocrtoc_ws/src/ocrtoc_motion_planning/data/"+model_name+"/visual_meshes/cloud.ply")
model_dir = pathlib.Path("/root/ocrtoc_materials/models/")
model_names = model_dir.glob('*')
model_names = [x.name for x in model_names if x.is_dir()]
#model_names = ['pudding_box']
for model_name in model_names:
    print('processing %s...' % (model_name))
    mesh_file_name = "/root/ocrtoc_materials/models/"+model_name+"/visual_meshes/visual.obj"
    save_path = "/root/ocrtoc_ws/src/ocrtoc_motion_planning/ocrtoc_motion_planning/data/models/"+model_name+"/visual_meshes/collision.obj"
    if not os.path.exists(mesh_file_name):
        continue
    if os.path.exists(save_path):
        continue
    # make directory
    if not os.path.exists("/root/ocrtoc_ws/src/ocrtoc_motion_planning/ocrtoc_motion_planning/data/models/"+model_name+"/visual_meshes/"):
        os.makedirs("/root/ocrtoc_ws/src/ocrtoc_motion_planning/ocrtoc_motion_planning/data/models/"+model_name+"/visual_meshes/")
    mesh = trimesh.load(mesh_file_name)
    # apply the transformation to the vertices
    vertices = np.array(mesh.vertices)
    #vertices_transform = np.ones((len(vertices), 4))
    #vertices_transform[:,:3] = vertices
    #vertices_transform = Re.dot(vertices_transform.T).T
    vertices_transform = Re.dot(vertices.T).T
    mesh.vertices = vertices_transform
    mesh.export(save_path)
    print('finished processing.')