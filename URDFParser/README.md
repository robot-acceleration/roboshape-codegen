# URDFParser

A simple parser libaray for URDF Files. That returns a ```robot``` object which can be used to access links, joints, transformation matricies, etc.

## Usage:
```python
parser = URDFParser()
robot = parser.parse(urdf_filepath, alpha_tie_breaker = False)
```
Where the tie breaker is used to order joints with the same parent link.
```python 
alpha_tie_breaker=False # URDF ordering used
alpha_tie_breaker=True # Joint name ordering used
```

## Dependencies:
There are 4 required packages ```beautifulsoup4, lxml, numpy, sympy``` which can be automatically installed by running:
```shell
pip3 install -r requirements.txt
```

## Robot API:

The main API is as follows where **XXX** can be replaced by:
+ **joint**: a joint object (see API below)
+ **link**: a link object (see API below)
+ **Xmat**: a sympy transformation matrix with one free variable as defined by its joint
+ **Xmat_Func**: a function that returns a numpy matrix when passed a value for the free variable
+ **Imat**: a numpy 6x6 inertia matrix
+ **S**: a numpy 6x1 motion subspace matrix

```python
# A single object by its ID or by its name as defined in the URDF
get_XXX_by_id(lid) # jid for joints 
get_XXX_by_name(name)
# A list of the objects that occur in the given bfs level
get_XXX_by_bfs_level(name)
# A list of the object ordered by their IDs or by their names as defined in the URDF
# Note: The base link/inertia exists at index -1 and so will appear at the beginning of the list
get_XXXs_ordered_by_id(reverse = False)
get_XXXs_ordered_by_name(reverse = False)
# A dictionary of objects by their ID or by their name as defined in the URDF
# Note: The base link/inertia exists at index -1
get_XXXs_dict_by_id()
get_XXXs_dict_by_name()
```

The API also includes the following functions:
```python
# get the number of positions and velocities in the robot state as well as numbers of links and joints
# note: links should be joints + 1 when including the base, num_joints = num_pos
#       num_vel = num_pos for fixed base (and is one larger with quaternion)
get_num_pos()
get_num_vel()
get_num_joints()
get_num_links()
get_num_links_effective() # num_links - 1 (base link is not used in many RBD algorithms when fixed)
# get the max bfs_level
get_max_bfs_level()
# get the IDs at a given bfs level
get_ids_by_bfs_level(level)
# get the ID of the parent of a given link by id
get_parent_id(lid)
# get the full list of parents ordered by id
get_parent_id_array()
# get the subtree IDs for a given link id
get_subtree_by_id(lid)
# get all joints that have parent link name as the parent or child link name as the child
get_joints_by_parent_name(parent_name)
get_joints_by_child_name(child_name)
# get the joint that has parent link name as the parent and child link name as the child
get_joint_by_parent_child_name(parent_name,child_name)
```

## Joint API:
```python
# get the name, id, and bfs of the joint
get_name()
get_id()
get_bfs_id()
get_bfs_level()
# get the parent and child link name
get_parent()
get_child()
# get the Xmat or Xmat_Func for this joint as defined above
get_transformation_matrix()
get_transformation_matrix_function()
# get the S for this joint as defined above
get_joint_subspace()
# get the velocity damping coefficent for this joint
get_damping()
```

## Link API:
```python
# get the name, id, and bfs of the link
get_name()
get_id()
get_bfs_id()
get_bfs_level()
# get the link's spatial inertia matrix
get_spatial_inertia()
```