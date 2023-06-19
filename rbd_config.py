import sys
import os

dim_list = ["AX", "AY", "AZ", "LX", "LY", "LZ"]
num_PEs = 6

try:
    automation_prefix = os.environ["AUTOMATION_PREFIX"]
except KeyError:
    print("AUTOMATION_PREFIX isn't set!")
    exit(1)

try:
    urdf_file = os.environ["URDF_FILE"]
except KeyError:
    #urdf_file = "iiwa_rbd_accel.urdf"
    #urdf_file = "iiwa_rbd_accel_pants.urdf"
    #urdf_file = "iiwa_brian_simple.urdf"
    #urdf_file = "iiwa_brian.urdf"
    #urdf_file = "iiwa_brian_simple_y.urdf"
    #urdf_file = "iiwa_brian_y.urdf"
    #urdf_file = "iiwa_brian_8_simple.urdf"
    #urdf_file = "hyq_simple.urdf"
    #urdf_file = "hyq_simple_xyz.urdf"
    #urdf_file = "hyq_simple_xyz_prismatic.urdf"
    #urdf_file = "iiwa_brian_simple_fixed.urdf"
    #urdf_file = "hyq_simple_fixed.urdf"
    #urdf_file = "root_fork_sabrina_8l_8r.urdf"
    #urdf_file = "iiwa_rbd_accel_9.urdf"
    #urdf_file = "iiwa_rbd_accel_8.urdf"
    #urdf_file = "iiwa_rbd_accel_6.urdf"
    #urdf_file = "iiwa_rbd_accel_5.urdf"
    #urdf_file = "pants.urdf"
    #urdf_file = "atlas.urdf"
    #urdf_file = "atlasSimple.urdf"
    #urdf_file = "atlasSimple2.urdf"
    urdf_file = "baxter_simple.urdf"

if urdf_file == "iiwa_rbd_accel.urdf" or urdf_file == "iiwa_rbd_accel_pants.urdf":
    block_size = 7
elif urdf_file == "hyq_simple.urdf":
    block_size = 6
elif urdf_file == "baxter_simple.urdf":
    block_size = 6
elif urdf_file == "atlas.urdf":
    block_size = 12
else:
    block_size = 7

if "/" not in urdf_file:
    # we provided only urdf filename, setting full path here
    urdf_file =  automation_prefix + "/urdfs/" + urdf_file

sys.path.insert(0, automation_prefix)
sys.path.insert(0, automation_prefix + "/util")
sys.path.insert(0, automation_prefix + "/URDFParser")
sys.path.insert(0, automation_prefix + "/rbdReference")
sys.path.insert(0, automation_prefix + "/FPGACodegen")
sys.path.insert(0, automation_prefix + "/GPUCodegen")
