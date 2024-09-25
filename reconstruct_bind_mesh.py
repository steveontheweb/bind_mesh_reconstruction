import maya.cmds as cmds
import maya.api.OpenMaya as om
import maya.api.OpenMayaAnim as oma
import time

def force_get_shape(node):
    if cmds.nodeType(node) == 'transform':
        return cmds.listRelatives(node, shapes=True)[0]
    else:
        return node

def get_dag_path(node_name):
    sel = om.MSelectionList()
    sel.add(node_name)
    return sel.getDagPath(0)

def get_mobject(node_name):
    sel = om.MSelectionList()
    sel.add(node_name)
    return sel.getDependNode(0)
    
def get_mfn_mesh(node_name):
    mobject = get_dag_path(node_name)
    return om.MFnMesh(mobject)

def get_skin_cluster_mobject(skin_cluster_name):
    sel_list = om.MSelectionList()
    sel_list.add(skin_cluster_name)
    return sel_list.getDependNode(0)

def get_skin_weights(skin_cluster_name):
    # Get the MObject for the skin cluster
    selection_list = om.MGlobal.getSelectionListByName(skin_cluster_name)
    skin_cluster_mobject = selection_list.getDependNode(0)
    
    # Initialize the skin cluster function set
    skin_fn = oma.MFnSkinCluster(skin_cluster_mobject)
    
    # Get connected geometry (assuming there's just one connected geometry)
    connected_geometries = cmds.listConnections(skin_cluster_name + '.outputGeometry[0]', s=False, d=True)
    if not connected_geometries:
        raise ValueError("No connected geometry found")
    
    first_connected_geometry = connected_geometries[0]
    
    # Get the dagPath for the geometry
    selection_list = om.MGlobal.getSelectionListByName(first_connected_geometry)
    dag_path = selection_list.getDagPath(0)
    
    # Getting the components (vertices in this case)
    components = om.MFnSingleIndexedComponent()
    components.create(om.MFn.kMeshVertComponent)
    
    # Get influence objects and weights
    influence_objects = skin_fn.influenceObjects()
    weights, num_influences = skin_fn.getWeights(dag_path, components.object())
    influence_indices = range(len(influence_objects))

    return weights, influence_indices

def set_skin_weights(skin_cluster_name, weights, influence_indices):
    # Get the MObject for the skin cluster
    selection_list = om.MGlobal.getSelectionListByName(skin_cluster_name)
    skin_cluster_mobject = selection_list.getDependNode(0)

    # Initialize the skin cluster function set
    skin_fn = oma.MFnSkinCluster(skin_cluster_mobject)

    # Get connected geometry (assuming there's just one connected geometry)
    connected_geometries = cmds.listConnections(skin_cluster_name + '.outputGeometry[0]', s=False, d=True)
    if not connected_geometries:
        raise ValueError("No connected geometry found")

    first_connected_geometry = connected_geometries[0]

    # Get the dagPath for the geometry
    selection_list = om.MGlobal.getSelectionListByName(first_connected_geometry)
    dag_path = selection_list.getDagPath(0)

    # Create a component object for vertices
    components = om.MFnSingleIndexedComponent()
    components.create(om.MFn.kMeshVertComponent)

    # Create an MDoubleArray for weights
    weight_array = om.MDoubleArray(weights)

    # Set the skin weights
    skin_fn.setWeights(dag_path, components.object(), om.MIntArray(influence_indices), weight_array)


def reconstruct_bind_mesh(input_mesh, output_mesh, bind_pose_time, deformed_time):
    """
    Given a mesh that was authored to be the "deformed mesh" at a particular deformation pose, we can 
    reconstruct the necessary bind mesh.  

    Note: in order to set up your scene for this, you'll need to set up the joints to animate from the 
    desired bind pose to the deformed pose at different times on the timeline.  The input mesh must be 
    properly skinned to the joints with the desired deformed pose as the input mesh's bind pose.
    
    Parameters:
    input_mesh (string): The name of the input mesh
    output_mesh (string): The name of the mesh that will be generated / reconstructed
    bind_pose_time (float): The position on the timeline with the desired bind pose of the mesh
    deformed_time (float): The position on the timeline with the deformed pose (probably the current bind pose)
    """
    start_time = time.time()

    input_mesh = force_get_shape(input_mesh)
    input_mesh_dag_path = get_dag_path(input_mesh)
    
    
    # Create an MFnMesh object for the input mesh
    input_mesh_fn = get_mfn_mesh(input_mesh)
    # Get the number of vertices in the input mesh
    num_vertices = input_mesh_fn.numVertices
    # Delete the output mesh if it already exists
    if cmds.objExists(output_mesh):
        cmds.delete(output_mesh)
    
    
    # Duplicate the input mesh
    duplicate_mesh = cmds.duplicate(input_mesh)[0]
    duplicate_mesh = cmds.rename(duplicate_mesh, output_mesh)
    duplicate_mesh_fn = get_mfn_mesh(duplicate_mesh)
    # List all the connections to the vertex attribute
    connections = cmds.listConnections(input_mesh, source=True, destination=False, plugs=True)
    # Find the skinCluster node among the connections
    skin_cluster_name = None
    for connection in connections:
        if cmds.nodeType(connection) == 'skinCluster':
            skin_cluster_name = connection.split('.')[0]
            break
    
    
    if skin_cluster_name:
        # Get the list of joints influencing the skin cluster
        joints = cmds.skinCluster(skin_cluster_name, query=True, inf=True)
        
        # Cache the bind pose
        # NOTE: we can't use the bind pose defined in the skin cluster because this mesh has the incorrect bind pose
        # so, we use bind_pose_time to get the real bind pose
        cmds.currentTime(bind_pose_time)
        cached_bind_pose = {}
        for joint in joints:
            m = om.MMatrix(cmds.xform(joint, q=True, matrix=True, worldSpace=True))
            cached_bind_pose[joint] = m

        # Now, make sure we're at the reference/accurate/deformed time, and cache the pose
        cmds.currentTime(deformed_time)
        cached_deformed_pose = {}
        for joint in joints:
            m = om.MMatrix(cmds.xform(joint, q=True, matrix=True, worldSpace=True))
            cached_deformed_pose[joint] = m      

        # get all source points
        source_points = input_mesh_fn.getPoints(space=om.MSpace.kWorld)

        # get all source weights
        skin_cluster = oma.MFnSkinCluster(get_mobject(skin_cluster_name))
               
        influence_objects = skin_cluster.influenceObjects()
        weights, num_influences = skin_cluster.getWeights(input_mesh_dag_path, om.MObject())
        weights = list(weights)
        
        bind_verts = []
        vertex_index = 0
        print("Reconstructing {} vertices".format(num_vertices))
        for i in range(0, len(weights), num_influences):
            cur_vertex = source_points[vertex_index]
            vertex_weights = weights[i:i + num_influences]
            weighted_skin_matrix = om.MMatrix([0.0] * 16)
            for j, weight in enumerate(vertex_weights):
                if weight > 0.0:
                    joint_name = joints[j]
                    current_joint_matrix = cached_deformed_pose[joint_name]                   
                    bind_joint_matrix = cached_bind_pose[joint_name]
                    skin_matrix = bind_joint_matrix.inverse() * current_joint_matrix
                    weighted_skin_matrix += weight * skin_matrix
            bind_vertex = cur_vertex * weighted_skin_matrix.inverse()
            bind_verts.append(bind_vertex)
            vertex_index += 1
            
        duplicate_mesh_fn.setPoints(bind_verts, space=om.MSpace.kWorld)
        
        # Now, we want to bind the new mesh to the original bind pose, and copy weights by index
        cmds.currentTime(bind_pose_time)
        cmds.select(clear=True)
        weights, influence_indices = get_skin_weights(skin_cluster_name)
        target_skin_cluster_name = cmds.skinCluster(joints, duplicate_mesh, toSelectedBones=True, bindMethod=0, skinMethod=0)[0]
        set_skin_weights(target_skin_cluster_name, weights, influence_indices)
     
    else:
        print("No skinCluster found for the mesh.")
        
    end_time = time.time()
    elapsed_time = end_time - start_time
    # currently, this is running about 15.7s
    print("reconstruct_bind_mesh() finished.  Elapsed time: {:.2f} seconds.".format(elapsed_time))
    

input_mesh = "pSphere2"
output_mesh = "{}_bindPoseMesh".format(input_mesh)
bind_pose_time = 0  # skeleton is accurate (proper bind pose) at this time, but the mesh will not be (improper bind pose)
ref_time = 30  # skeleton and the mesh are accurate at this time (deformed)

reconstruct_bind_mesh(input_mesh, output_mesh, bind_pose_time, ref_time)
