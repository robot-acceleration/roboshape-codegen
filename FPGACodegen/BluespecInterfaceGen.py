import numpy as np
import time

class BluespecInterfaceGen:
    def __init__(self, num_PEs, block_size, robot):
        self.num_PEs = num_PEs
        self.block_size = block_size
        self.dim_list = ["AX","AY","AZ","LX","LY","LZ"]
        self.robot = robot

    
    def get_robot_limbs(self):
        robot = self.robot
        robot_joints = robot.get_joints_ordered_by_id()

        limbs = []
        curr_limb = [robot_joints[0].get_name()]

        for curr_joint in robot_joints[1:]:
            curr_joint_name = curr_joint.get_name()
            curr_jid = curr_joint.get_id()
            parent_link = curr_joint.get_parent()
            parent_joints = robot.get_joints_by_child_name(parent_link)
            if parent_joints:
                parent_joint = parent_joints[0]
                parent_jid = parent_joint.get_id()
                if parent_jid == curr_jid-1:
                    curr_limb.append(curr_joint_name)
                else:
                    limbs.append(curr_limb)
                    curr_limb = []
                    curr_limb.append(curr_joint_name)
            else:
                limbs.append(curr_limb)
                curr_limb = []
                curr_limb.append(curr_joint_name)

        limbs.append(curr_limb)
        return limbs

    def get_robot_limb_preds(self, limbs):
        robot = self.robot

        preds_limbs = []
        for limb in limbs:
            preds_limb = []
            limb_base_joint = robot.get_joint_by_name(limb[0])

            curr_joint = limb_base_joint
            parent_link = curr_joint.get_parent()
            parent_joints = robot.get_joints_by_child_name(parent_link)

            while parent_joints:
                parent_joint = parent_joints[0]
                preds_limb.append(parent_joint.get_name())

                curr_joint = parent_joint
                parent_link = curr_joint.get_parent()
                parent_joints = robot.get_joints_by_child_name(parent_link)
            preds_limbs.append(preds_limb)
        
        return preds_limbs

    def get_parents_chain(self, chain):
        robot = self.robot

        chain_par = []
        for i,joint_name in enumerate(chain):
            if joint_name == "world":
                # next joint will be the parent since world only appears in the
                # beginning of each backward pass chain
                chain_par.append(chain[i+1])
                continue
            curr_joint = robot.get_joint_by_name(joint_name)
            parent_link = curr_joint.get_parent()
            parent_joints = robot.get_joints_by_child_name(parent_link)

            if parent_joints:
                parent_joint = parent_joints[0]
                chain_par.append(parent_joint.get_name())
            else:
                chain_par.append("nop")
        return chain_par

    def gen_robot_limb_fproc_chains(self):
        robot = self.robot
        
        limbs = self.get_robot_limbs()
        preds_limbs = self.get_robot_limb_preds(limbs)

        n_limbs = len(limbs)

        L_ID_pars = []
        L_ID_names = []
        L_ID_dists = []

        L_dL_i_pars = []
        L_dL_i_names = []
        L_dL_i_dervs = []
        L_dL_i_dists = []

        for limb_id in range(n_limbs):
            print("Limb: " + str(limb_id))

            limb = limbs[limb_id]
            preds_limb = preds_limbs[limb_id]

            L_ID_name = limb
            L_ID_par = self.get_parents_chain(L_ID_name)
            L_ID_dist = list(range(2, len(L_ID_name)+2))[::-1]

            L_ID_names.append(L_ID_name)
            L_ID_pars.append(L_ID_par)
            L_ID_dists.append(L_ID_dist)

            print("ID:")
            print(L_ID_par)
            print(L_ID_name)
            print(L_ID_dist)

            print("L_dL for preds_limb:")
            print("preds_limb: " + str(preds_limb))
            for pred_limb in preds_limb:
                L_dL_i_name = limb
                L_dL_i_par = self.get_parents_chain(L_dL_i_name)
                L_dL_i_derv = [pred_limb] * len(L_dL_i_name)
                L_dL_i_dist = list(range(1, len(L_dL_i_name)+1))[::-1]

                L_dL_i_names.append(L_dL_i_name)
                L_dL_i_pars.append(L_dL_i_par)
                L_dL_i_dervs.append(L_dL_i_derv)
                L_dL_i_dists.append(L_dL_i_dist)

                print(L_dL_i_par)
                print(L_dL_i_name)
                print(L_dL_i_derv)
                print(L_dL_i_dist)

            print("L_dL for limb:")
            len_limb = len(limb)
            for i in range(len_limb):
                L_dL_i_name = limb[i:]
                L_dL_i_par = self.get_parents_chain(L_dL_i_name)
                L_dL_i_derv = [limb[i]] * len(L_dL_i_name)
                L_dL_i_dist = list(range(1, len(L_dL_i_name)+1))[::-1]

                L_dL_i_names.append(L_dL_i_name)
                L_dL_i_pars.append(L_dL_i_par)
                L_dL_i_dervs.append(L_dL_i_derv)
                L_dL_i_dists.append(L_dL_i_dist)

                print(L_dL_i_par)
                print(L_dL_i_name)
                print(L_dL_i_derv)
                print(L_dL_i_dist)

            print("--------")
        return {
                "L_ID_pars": L_ID_pars,
                "L_ID_names": L_ID_names,
                "L_ID_dists": L_ID_dists,

                "L_dL_i_pars": L_dL_i_pars,
                "L_dL_i_names": L_dL_i_names,
                "L_dL_i_dervs": L_dL_i_dervs,
                "L_dL_i_dists": L_dL_i_dists,
        }

    def gen_robot_limb_bproc_chains(self):
        robot = self.robot
        
        limbs = self.get_robot_limbs()
        preds_limbs = self.get_robot_limb_preds(limbs)

        # some processing for backward pass
        rev_limbs = []
        for limb in limbs:
            rev_limb = limb[::-1]
            rev_limb.insert(0, "world")
            rev_limbs.append(rev_limb)
        limbs = rev_limbs

        n_limbs = len(limbs)

        L_ID_pars = []
        L_ID_names = []
        L_ID_dists = []

        L_dL_i_pars = []
        L_dL_i_names = []
        L_dL_i_dervs = []
        L_dL_i_dists = []

        for limb_id in range(n_limbs):
            print("Limb: " + str(limb_id))

            limb = limbs[limb_id]
            preds_limb = preds_limbs[limb_id]

            L_ID_name = limb
            L_ID_par = self.get_parents_chain(L_ID_name)
            L_ID_dist = list(range(2, len(L_ID_name)+2))[::-1]

            L_ID_names.append(L_ID_name)
            L_ID_pars.append(L_ID_par)
            L_ID_dists.append(L_ID_dist)

            print("ID:")
            print("par: " + str(L_ID_par))
            print("name: " + str(L_ID_name))
            print("dist: " + str(L_ID_dist))

            print("L_dL for preds_limb:")
            print("preds_limb: " + str(preds_limb))
            for pred_limb in preds_limb:
                L_dL_i_name = limb
                L_dL_i_par = self.get_parents_chain(L_dL_i_name)
                L_dL_i_derv = [pred_limb] * len(L_dL_i_name)
                L_dL_i_dist = list(range(1, len(L_dL_i_name)+1))[::-1]

                L_dL_i_names.append(L_dL_i_name)
                L_dL_i_pars.append(L_dL_i_par)
                L_dL_i_dervs.append(L_dL_i_derv)
                L_dL_i_dists.append(L_dL_i_dist)

                print("pred par: " + str(L_dL_i_par))
                print("pred name: " + str(L_dL_i_name))
                print("pred derv: " + str(L_dL_i_derv))
                print("pred dist: " + str(L_dL_i_dist))

            print("L_dL for limb:")
            len_limb = len(limb)
            for i in range(1,len_limb):
                L_dL_i_name = limb
                L_dL_i_par = self.get_parents_chain(L_dL_i_name)
                L_dL_i_derv = [L_dL_i_name[i]] * len(L_dL_i_name)
                L_dL_i_dist = list(range(1, len(L_dL_i_name)+1))[::-1]

                L_dL_i_names.append(L_dL_i_name)
                L_dL_i_pars.append(L_dL_i_par)
                L_dL_i_dervs.append(L_dL_i_derv)
                L_dL_i_dists.append(L_dL_i_dist)

                print("par: " + str(L_dL_i_par))
                print("name: " + str(L_dL_i_name))
                print("derv: " + str(L_dL_i_derv))
                print("dist: " + str(L_dL_i_dist))

            print("--------")
        return {
                "L_ID_pars": L_ID_pars,
                "L_ID_names": L_ID_names,
                "L_ID_dists": L_ID_dists,

                "L_dL_i_pars": L_dL_i_pars,
                "L_dL_i_names": L_dL_i_names,
                "L_dL_i_dervs": L_dL_i_dervs,
                "L_dL_i_dists": L_dL_i_dists,
        }

    def hus_algorithm_scheduling_fproc(self):
        ID_GradID_lists = self.gen_robot_limb_fproc_chains()
        all_ID_par_lists = ID_GradID_lists["L_ID_pars"]
        all_ID_name_lists = ID_GradID_lists["L_ID_names"]
        all_ID_dist_lists = ID_GradID_lists["L_ID_dists"]

        all_GradID_par_lists = ID_GradID_lists["L_dL_i_pars"]
        all_GradID_name_lists = ID_GradID_lists["L_dL_i_names"]
        all_GradID_derv_lists = ID_GradID_lists["L_dL_i_dervs"]
        all_GradID_dist_lists = ID_GradID_lists["L_dL_i_dists"]

        self.hus_algorithm_scheduling(all_ID_par_lists, all_ID_name_lists, all_ID_dist_lists, \
            all_GradID_par_lists, all_GradID_name_lists, all_GradID_derv_lists, all_GradID_dist_lists)

    def hus_algorithm_scheduling_bproc(self):
        ID_GradID_lists = self.gen_robot_limb_bproc_chains()
        all_ID_par_lists = ID_GradID_lists["L_ID_pars"]
        all_ID_name_lists = ID_GradID_lists["L_ID_names"]
        all_ID_dist_lists = ID_GradID_lists["L_ID_dists"]

        all_GradID_par_lists = ID_GradID_lists["L_dL_i_pars"]
        all_GradID_name_lists = ID_GradID_lists["L_dL_i_names"]
        all_GradID_derv_lists = ID_GradID_lists["L_dL_i_dervs"]
        all_GradID_dist_lists = ID_GradID_lists["L_dL_i_dists"]

        #self.hus_algorithm_scheduling(all_ID_par_lists, all_ID_name_lists, all_ID_dist_lists, \
        #    all_GradID_par_lists, all_GradID_name_lists, all_GradID_derv_lists, all_GradID_dist_lists)

    def hus_algorithm_scheduling(self, all_ID_par_lists, all_ID_name_lists, all_ID_dist_lists, \
            all_GradID_par_lists, all_GradID_name_lists, all_GradID_derv_lists, all_GradID_dist_lists):

        #-------------------------------------------------------------------------------
        # Scheduling based on Hu's Algorithm
        #-------------------------------------------------------------------------------
        num_PEs = self.num_PEs

        # Initialize bookkeeping
        completed_ID_node_par_list  = ["nop"]
        completed_ID_node_name_list = ["nop"]
        completed_ID_node_dist_list = [0]
        all_ID_nodes_scheduled = False

        allPEs_completed_GradID_node_par_list  = []
        allPEs_completed_GradID_node_name_list = []
        allPEs_completed_GradID_node_derv_list = []
        allPEs_completed_GradID_node_dist_list = []
        for PE_id in range(0,num_PEs):
           allPEs_completed_GradID_node_par_list.append(["nop"])
           allPEs_completed_GradID_node_name_list.append(["nop"])
           allPEs_completed_GradID_node_derv_list.append(["nop"])
           allPEs_completed_GradID_node_dist_list.append([0])
        all_GradID_nodes_scheduled = False

        while ((all_ID_nodes_scheduled == False)|(all_GradID_nodes_scheduled == False)):
           # Find the ID list with the node with max dist
           curr_max_ID_dist = 0
           curr_max_dist_ID_list_index = 0
           for ID_list_index,ID_list in enumerate(all_ID_name_lists):
              if (len(ID_list) != 0):
                 ID_list_par  = all_ID_par_lists[ID_list_index]
                 ID_list_name = all_ID_name_lists[ID_list_index]
                 ID_list_dist = all_ID_dist_lists[ID_list_index]
                 ID_node_par  = ID_list_par[0]
                 ID_node_name = ID_list_name[0]
                 ID_node_dist = ID_list_dist[0]
                 ###print("ID: (",ID_node_par,",",ID_node_name,",",ID_node_dist,")")
                 if (ID_node_dist > curr_max_ID_dist):
                    prereqs_met = False
                    # Check that the prereqs are met
                    for completed_ID_node_name in completed_ID_node_name_list:
                       if (completed_ID_node_name == ID_node_par):
                          prereqs_met = True
                    if (prereqs_met == True):
                       # Save the node info
                       curr_max_ID_dist = ID_list_dist[0]
                       curr_max_dist_ID_list_index = ID_list_index
           ###if (curr_max_ID_dist == 0):
              ###print("Error: No Max ID Dist found. Current Max ID Dist:",curr_max_ID_dist,"List Index:",curr_max_dist_ID_list_index)
           ###else:
              ###print("Max ID Dist Found: Current Max ID Dist:",curr_max_ID_dist,"List Index:",curr_max_dist_ID_list_index)

           # Schedule as many GradGradID nodes as possible with the max dists
           for PE_id in range(0,num_PEs):
              # Find the GradID list with the node with max dist
              curr_max_GradID_dist = 0
              curr_max_dist_GradID_list_index = 0
              for GradID_list_index,GradID_list in enumerate(all_GradID_name_lists):
                 if (len(GradID_list) != 0):
                    GradID_list_par  = all_GradID_par_lists[GradID_list_index]
                    GradID_list_name = all_GradID_name_lists[GradID_list_index]
                    GradID_list_derv = all_GradID_derv_lists[GradID_list_index]
                    GradID_list_dist = all_GradID_dist_lists[GradID_list_index]
                    GradID_node_par  = GradID_list_par[0]
                    GradID_node_name = GradID_list_name[0]
                    GradID_node_derv = GradID_list_derv[0]
                    GradID_node_dist = GradID_list_dist[0]
                    ###print("GradID: (",GradID_node_par,",",GradID_node_name,",",GradID_node_dist,")")
                    if (GradID_node_dist > curr_max_GradID_dist):
                       ID_prereqs_met = False
                       GradID_prereqs_met = False
                       # Check that the ID prereqs are met
                       for completed_ID_node_name in completed_ID_node_name_list:
                          if (completed_ID_node_name == GradID_node_name):
                             ID_prereqs_met = True
                       # Check that the GradID prereqs are met
                       for completed_GradID_node_name_list in allPEs_completed_GradID_node_name_list:
                          for completed_GradID_node_name in completed_GradID_node_name_list:
                             if (completed_GradID_node_name == GradID_node_par):
                                GradID_prereqs_met = True
                       if (ID_prereqs_met == True)&(GradID_prereqs_met == True):
                          # Save the node info
                          curr_max_GradID_dist = GradID_list_dist[0]
                          curr_max_dist_GradID_list_index = GradID_list_index
              if (curr_max_GradID_dist == 0):
                 ###print("Error: No Max GradID Dist found. Current Max GradID Dist:",curr_max_GradID_dist,"List Index:",curr_max_dist_GradID_list_index)
                 # Schedule a NOP for this PE
                 completed_GradID_node_par_list  = allPEs_completed_GradID_node_par_list[PE_id]
                 completed_GradID_node_name_list = allPEs_completed_GradID_node_name_list[PE_id]
                 completed_GradID_node_derv_list = allPEs_completed_GradID_node_derv_list[PE_id]
                 completed_GradID_node_dist_list = allPEs_completed_GradID_node_dist_list[PE_id]      
                 completed_GradID_node_par_list.append("nop")
                 completed_GradID_node_name_list.append("nop")
                 completed_GradID_node_derv_list.append("nop")
                 completed_GradID_node_dist_list.append(0)
              else:
                 ###print("Max GradID Dist Found: Current Max GradID Dist:",curr_max_GradID_dist,"List Index:",curr_max_dist_GradID_list_index)
                 # Schedule that GradID node and remove it from its list
                 curr_max_dist_GradID_par_list  = all_GradID_par_lists[curr_max_dist_GradID_list_index]
                 curr_max_dist_GradID_name_list = all_GradID_name_lists[curr_max_dist_GradID_list_index]
                 curr_max_dist_GradID_derv_list = all_GradID_derv_lists[curr_max_dist_GradID_list_index]
                 curr_max_dist_GradID_dist_list = all_GradID_dist_lists[curr_max_dist_GradID_list_index]
                 curr_max_dist_GradID_node_par  = curr_max_dist_GradID_par_list[0]
                 curr_max_dist_GradID_node_name = curr_max_dist_GradID_name_list[0]
                 curr_max_dist_GradID_node_derv = curr_max_dist_GradID_derv_list[0]
                 curr_max_dist_GradID_node_dist = curr_max_dist_GradID_dist_list[0]
                 # Choose a PE and add GradID node to schedule
                 completed_GradID_node_par_list  = allPEs_completed_GradID_node_par_list[PE_id]
                 completed_GradID_node_name_list = allPEs_completed_GradID_node_name_list[PE_id]
                 completed_GradID_node_derv_list = allPEs_completed_GradID_node_derv_list[PE_id]
                 completed_GradID_node_dist_list = allPEs_completed_GradID_node_dist_list[PE_id]      
                 completed_GradID_node_par_list.append(curr_max_dist_GradID_node_par)
                 completed_GradID_node_name_list.append(curr_max_dist_GradID_node_name)
                 completed_GradID_node_derv_list.append(curr_max_dist_GradID_node_derv)
                 completed_GradID_node_dist_list.append(curr_max_dist_GradID_node_dist)
                 # Remove that GradID node from its list
                 curr_max_dist_GradID_par_list.pop(0)
                 curr_max_dist_GradID_name_list.pop(0)
                 curr_max_dist_GradID_derv_list.pop(0)
                 curr_max_dist_GradID_dist_list.pop(0)

           # Schedule that ID node and remove it from its list
           if (curr_max_ID_dist != 0):
              curr_max_dist_ID_par_list  = all_ID_par_lists[curr_max_dist_ID_list_index]
              curr_max_dist_ID_name_list = all_ID_name_lists[curr_max_dist_ID_list_index]
              curr_max_dist_ID_dist_list = all_ID_dist_lists[curr_max_dist_ID_list_index]
              curr_max_dist_ID_node_par  = curr_max_dist_ID_par_list[0]
              curr_max_dist_ID_node_name = curr_max_dist_ID_name_list[0]
              curr_max_dist_ID_node_dist = curr_max_dist_ID_dist_list[0]
              completed_ID_node_par_list.append(curr_max_dist_ID_node_par)
              completed_ID_node_name_list.append(curr_max_dist_ID_node_name)
              completed_ID_node_dist_list.append(curr_max_dist_ID_node_dist)
              curr_max_dist_ID_par_list.pop(0)
              curr_max_dist_ID_name_list.pop(0)
              curr_max_dist_ID_dist_list.pop(0)
           
           # Check if all ID nodes scheduled
           all_ID_nodes_scheduled = True
           for ID_list_index,ID_list in enumerate(all_ID_name_lists):
              if (len(ID_list) != 0):
                 all_ID_nodes_scheduled = False

           # Check if all GradID nodes scheduled
           all_GradID_nodes_scheduled = True
           for GradID_list_index,GradID_list in enumerate(all_GradID_name_lists):
              if (len(GradID_list) != 0):
                 all_GradID_nodes_scheduled = False
                 
        # Remove the Initial NOPs from the ID schedule lists
        completed_ID_node_par_list.pop(0)
        completed_ID_node_name_list.pop(0)
        completed_ID_node_dist_list.pop(0)

        # Remove the Initial NOPs from the GradID schedule lists
        for PE_id in range(0,num_PEs):
           completed_GradID_node_par_list  = allPEs_completed_GradID_node_par_list[PE_id]
           completed_GradID_node_name_list = allPEs_completed_GradID_node_name_list[PE_id]
           completed_GradID_node_derv_list = allPEs_completed_GradID_node_derv_list[PE_id]
           completed_GradID_node_dist_list = allPEs_completed_GradID_node_dist_list[PE_id]
           completed_GradID_node_par_list.pop(0)
           completed_GradID_node_name_list.pop(0)
           completed_GradID_node_derv_list.pop(0)
           completed_GradID_node_dist_list.pop(0)

        # Print ID schedule lists
        ###print("ID Parent:  ",completed_ID_node_par_list)
        ###print("ID Current: ",completed_ID_node_name_list)
        ###print("ID Distance:",completed_ID_node_dist_list)

        # Print GradID schedule lists
        ###for PE_id in range(0,num_PEs):
           ###print("[PE",PE_id,"] GradID Parent:  ",allPEs_completed_GradID_node_par_list[PE_id])
           ###print("[PE",PE_id,"] GradID Current: ",allPEs_completed_GradID_node_name_list[PE_id])
           ###print("[PE",PE_id,"] GradID Deriv.:  ",allPEs_completed_GradID_node_derv_list[PE_id])
           ###print("[PE",PE_id,"] GradID Distance:",allPEs_completed_GradID_node_dist_list[PE_id])

        #-------------------------------------------------------------------------------
        # Formatting for BluespecInterfaceGen.py
        #-------------------------------------------------------------------------------

        # Number link based on Assumed Input Joint Configuration Ordering
        joint_config_order_list = [joint.get_name() for joint in self.robot.get_joints_ordered_by_id()]
        joint_config_order_list.insert(0,"nop")
        joint_config_order_list.append("world")

        # While numbering links, search for max schedule length
        len_fproc_sched = 0

        # Number links for ID lists
        for node_index,completed_ID_node_par in enumerate(completed_ID_node_par_list):
           for joint_index,joint_name in enumerate(joint_config_order_list):
              if (completed_ID_node_par == joint_name):
                 completed_ID_node_par_list[node_index] = joint_index
        for node_index,completed_ID_node_name in enumerate(completed_ID_node_name_list):
           for joint_index,joint_name in enumerate(joint_config_order_list):
              if (completed_ID_node_name == joint_name):
                 completed_ID_node_name_list[node_index] = joint_index
        len_ID_sched = len(completed_ID_node_name_list)
        if (len_ID_sched > len_fproc_sched):
           len_fproc_sched = len_ID_sched

        # Number links for ID lists
        for PE_id in range(0,num_PEs):
           completed_GradID_node_par_list  = allPEs_completed_GradID_node_par_list[PE_id]
           completed_GradID_node_name_list = allPEs_completed_GradID_node_name_list[PE_id]
           completed_GradID_node_derv_list = allPEs_completed_GradID_node_derv_list[PE_id]
           completed_GradID_node_dist_list = allPEs_completed_GradID_node_dist_list[PE_id]
           for node_index,completed_GradID_node_par in enumerate(completed_GradID_node_par_list):
              for joint_index,joint_name in enumerate(joint_config_order_list):
                 if (completed_GradID_node_par == joint_name):
                    completed_GradID_node_par_list[node_index] = joint_index
           for node_index,completed_GradID_node_name in enumerate(completed_GradID_node_name_list):
              for joint_index,joint_name in enumerate(joint_config_order_list):
                 if (completed_GradID_node_name == joint_name):
                    completed_GradID_node_name_list[node_index] = joint_index
           for node_index,completed_GradID_node_derv in enumerate(completed_GradID_node_derv_list):
              for joint_index,joint_name in enumerate(joint_config_order_list):
                 if (completed_GradID_node_derv == joint_name):
                    completed_GradID_node_derv_list[node_index] = joint_index
           len_GradID_sched = len(completed_GradID_node_name_list)
           if (len_GradID_sched > len_fproc_sched):
              len_fproc_sched = len_GradID_sched

        # Print ID schedule lists
        print("ID Parent:  ",completed_ID_node_par_list)
        print("ID Current: ",completed_ID_node_name_list)
        print("ID Distance:",completed_ID_node_dist_list)

        # Print GradID schedule lists
        for PE_id in range(0,num_PEs):
           print("[PE",PE_id,"] GradID Parent:  ",allPEs_completed_GradID_node_par_list[PE_id])
           print("[PE",PE_id,"] GradID Current: ",allPEs_completed_GradID_node_name_list[PE_id])
           print("[PE",PE_id,"] GradID Deriv.:  ",allPEs_completed_GradID_node_derv_list[PE_id])
           print("[PE",PE_id,"] GradID Distance:",allPEs_completed_GradID_node_dist_list[PE_id])

        # Assemble the output lists
        fproc_curr_sched = []
        fproc_par_sched = []
        fproc_derv_sched = []
        for row in range(0,len_fproc_sched):
           curr_row = []
           par_row  = []
           derv_row = []
           for PE_id in range(0,num_PEs):
              completed_GradID_node_curr_list = allPEs_completed_GradID_node_name_list[PE_id]
              completed_GradID_node_par_list  = allPEs_completed_GradID_node_par_list[PE_id]
              completed_GradID_node_derv_list = allPEs_completed_GradID_node_derv_list[PE_id]
              curr_row.append(completed_GradID_node_curr_list[0])
              par_row.append(completed_GradID_node_par_list[0])
              derv_row.append(completed_GradID_node_derv_list[0])
              if (len(completed_GradID_node_curr_list) != 0):
                 completed_GradID_node_curr_list.pop(0)
                 completed_GradID_node_par_list.pop(0)
                 completed_GradID_node_derv_list.pop(0)
           fproc_curr_sched.append(curr_row)
           fproc_par_sched.append(par_row)
           fproc_derv_sched.append(derv_row)

        # Print formatted output
        print("sched_table_dict = {")
        print("                \"len_fproc_sched\": "+str(len_fproc_sched)+",")
        print("                \"rnea_fproc_curr_sched\": "+str(completed_ID_node_name_list)+", ")
        print("                \"rnea_fproc_par_sched\":  "+str(completed_ID_node_par_list)+", ")
        print("                \"fproc_curr_sched\": [")
        for row_id,row in enumerate(fproc_curr_sched):
           if ((row_id+1) < len(fproc_curr_sched)):
              print("                        "+str(row)+",")
           else:
              print("                        "+str(row))
        print("                ],")
        print("                \"fproc_par_sched\":  [")
        for row_id,row in enumerate(fproc_par_sched):
           if ((row_id+1) < len(fproc_par_sched)):
              print("                        "+str(row)+",")
           else:
              print("                        "+str(row))
        print("                ],")
        print("                \"fproc_derv_sched\": [")
        for row_id,row in enumerate(fproc_derv_sched):
           if ((row_id+1) < len(fproc_derv_sched)):
              print("                        "+str(row)+",")
           else:
              print("                        "+str(row))
        print("                ],")
        print("            }")


    def get_branch_parent_lids(self):
        robot = self.robot

        print("Links (minus root link):")
        print([l.get_name() for l in robot.get_links_ordered_by_id()][1:])
        print([l.get_id()+1 for l in robot.get_links_ordered_by_id()][1:])

        print("Joints")
        print([j.get_name() for j in robot.get_joints_ordered_by_id()][1:])
        print([j.get_id() for j in robot.get_joints_ordered_by_id()][1:])
        
        branch_parents_lids = []

        num_links = robot.get_num_links_effective()
        for lid in range(num_links):
            print("lid: " + str(lid))
            link = robot.get_link_by_id(lid)
            children_joints = robot.get_joints_by_parent_name(link.get_name())
            print(link.get_name())
            print([child.get_name() for child in children_joints])
            if len(children_joints) > 1:
                branch_parents_lids.append(lid+1)

        return branch_parents_lids


    def get_children_joints_by_parent_joint(self, parent_joint):
        child_link = parent_joint.get_child()
        children_joints = self.robot.get_joints_by_parent_name(child_link)
        return children_joints

    def get_fproc_schedule_tables_PE(self):

        print("fproc_curr_sched: ")
        print(fproc_curr_sched)
        print("fproc_derv_sched: ")
        print(fproc_derv_sched)

        return {
                "len_fproc_sched": len_fproc_sched,
                "fproc_curr_sched": fproc_curr_sched, 
                "fproc_derv_sched": fproc_derv_sched
        }

    #===============================================
    # MATMUL STUFF
    #===============================================

    def get_config(self):
        num_joints = self.robot.get_num_links_effective()

        # allocate memory
        q = np.zeros((num_joints))
        qd = np.zeros((num_joints))
        u = np.zeros((num_joints))
        qdd = np.zeros((num_joints))

        # load CPP rand point
        if num_joints > 0:
            q[0] = -0.336899
            qd[0] = 0.43302
            u[0] = 0.741788
        if num_joints > 1:
            q[1] = 1.29662
            qd[1] = -0.421561
            u[1] = 1.92844
        if num_joints > 2:
            q[2] = -0.677475 
            qd[2] = -0.645439
            u[2] = -0.903882
        if num_joints > 3:
            q[3] = -1.42182
            qd[3] = -1.86055
            u[3] = 0.0333959
        if num_joints > 4:
            q[4] = -0.706676
            qd[4] = -0.0130938
            u[4] = 1.17986
        if num_joints > 5:
            q[5] = -0.134981 
            qd[5] = -0.458284
            u[5] = -1.94599
        if num_joints > 6:
            q[6] = -1.14953
            qd[6] = 0.741174
            u[6] = 0.32869
        if num_joints > 7:
            q[7] = -0.296646
            qd[7] = 1.76642
            u[7] = -0.139457
        if num_joints > 8:
            q[8] = 2.13845
            qd[8] = 0.898011
            u[8] = 2.00667
        if num_joints > 9:
            q[9] = 2.00956
            qd[9] = -1.85675
            u[9] = -0.519292
        if num_joints > 10:
            q[10] = 1.55163
            qd[10] = 1.62223
            u[10] = -0.711198
        if num_joints > 11:
            q[11] = 2.2893
            qd[11] = 0.709379
            u[11] = 0.376638
        if num_joints > 12:
            q[12] = 0.0418005
            qd[12] = -0.382885
            u[12] = -0.209225
        if num_joints > 13:
            q[13] = -0.125271
            qd[13] = -0.239602
            u[13] = -0.816928
        if num_joints > 14:
            q[14] = -1.35512
            qd[14] = 1.88499
            u[14] = -0.943019
        if num_joints > 15:
            q[15] = -0.606463
            qd[15] = -2.20784
            u[15] = -2.16433
        if num_joints > 16:
            q[16] = -2.13552
            qd[16] = -0.921183
            u[16] = 1.37954
        if num_joints > 17:
            q[17] = 0.229695
            qd[17] = -0.110463
            u[17] = 0.456738
        if num_joints > 18:
            q[18] = 0.229592
            qd[18] = -1.64542
            u[18] = -0.702506
        if num_joints > 19:
            q[19] = -0.197398
            qd[19] = -1.7481
            u[19] = 0.159814
        if num_joints > 20:
            q[20] = -0.221438
            qd[20] = -0.562579
            u[20] = 0.944469
        if num_joints > 21:
            q[21] = 1.02441
            qd[21] = 1.02289
            u[21] = 0.100297
        if num_joints > 22:
            q[22] = -0.9309
            qd[22] = 0.21233
            u[22] = -0.1311
        if num_joints > 23:
            q[23] = 1.12961
            qd[23] = 1.30624
            u[23] = 0.750389
        if num_joints > 24:
            q[24] = 0.864741
            qd[24] = 1.31059
            u[24] = -0.666778
        if num_joints > 25:
            q[25] = 0.705222
            qd[25] = -0.0383565
            u[25] = 0.486885
        if num_joints > 26:
            q[26] = 0.0810176
            qd[26] = 0.317353
            u[26] = 0.513445
        if num_joints > 27:
            q[27] = 0.541962
            qd[27] = 0.479234
            u[27] = 0.0573834
        if num_joints > 28:
            q[28] = 1.01213
            qd[28] = 0.55686
            u[28] = 0.425883
        if num_joints > 29:
            q[29] = 2.213
            qd[29] = 0.541122
            u[29] = 0.293804
        if num_joints > 30:
            print("[!ERROR] CPP Random Match only implemented up to n = 30. Please use a lower dof URDF.")
            exit()
        
        return q, qd, qdd, u, num_joints

    #def gen_tiled_matmul(self):
    #    block_size = self.block_size
    #    rbd_reference = rbdReference(self.robot)

    #    q, qd, qdd, u, num_links = self.get_config()
    #    Minv = rbd_reference.minv(q)

    #    n_tiles = 0
    #    len_block_minv_sched_per_matrix = 0

    #    lh_tile_sched = []
    #    rh_tile_sched = []
    #    output_tile_incr_sched = []
    #    rh_tile_col_sched = [] # len_sched x num_PEs

    #    lh_tile_row_start_sched = []
    #    lh_tile_col_start_sched = []
    #    rh_tile_row_start_sched = []
    #    rh_tile_col_start_sched = []
    #    output_tile_row_start_sched = []
    #    output_tile_col_start_sched = []

    #    


    #===============================================
    # Schedule data structures
    #===============================================

    def get_fproc_schedule_tables(self):
        #schedules_PE = self.get_fproc_schedule_tables_PE()
        #schedules_rnea = self.get_fproc_schedule_tables_rnea()

        # temporary for hard-coded schedule tables
        sched_table_dict = None

        if self.robot.get_num_joints() == 7 and len(self.get_branch_parent_lids()) == 0 and self.num_PEs == 7:
            # iiwa-7
            sched_table_dict = {
                "len_fproc_sched": 8,
                "rnea_fproc_curr_sched": [1,2,3,4,5,6,7,0], 
                "rnea_fproc_par_sched":  [0,1,2,3,4,5,6,7], 
                "fproc_curr_sched": [
                        [0,0,0,0,0,0,0],
                        [1,0,0,0,0,0,0],
                        [2,2,0,0,0,0,0],
                        [3,3,3,0,0,0,0],
                        [4,4,4,4,0,0,0],
                        [5,5,5,5,5,0,0],
                        [6,6,6,6,6,6,0],
                        [7,7,7,7,7,7,7]
                ],
                "fproc_par_sched": [
                        [0,0,0,0,0,0,0],
                        [0,0,0,0,0,0,0],
                        [1,1,0,0,0,0,0],
                        [2,2,2,0,0,0,0],
                        [3,3,3,3,0,0,0],
                        [4,4,4,4,4,0,0],
                        [5,5,5,5,5,5,0],
                        [6,6,6,6,6,6,6]
                ],
                "fproc_derv_sched": [ 
                        [1,2,3,4,5,6,7],
                        [1,2,3,4,5,6,7],
                        [1,2,3,4,5,6,7],
                        [1,2,3,4,5,6,7],
                        [1,2,3,4,5,6,7],
                        [1,2,3,4,5,6,7],
                        [1,2,3,4,5,6,7],
                        [1,2,3,4,5,6,7]
                ]
            }
        elif self.robot.get_num_joints() == 7 and len(self.get_branch_parent_lids()) == 1 and self.num_PEs == 7:
            # iiwa-pants
            sched_table_dict = {
                "len_fproc_sched": 8,
                "rnea_fproc_curr_sched": [1,2,3,4,5,6,7,0], 
                "rnea_fproc_par_sched":  [0,1,2,3,1,5,6,7], 
                "fproc_curr_sched": [
                        [0,0,0,0,0,0,0],
                        [1,0,0,0,0,0,0],
                        [2,2,0,0,0,0,0],
                        [3,3,3,0,0,0,0],
                        [4,4,4,4,0,0,0],
                        [5,0,0,0,5,0,0],
                        [6,0,0,0,6,6,0],
                        [7,0,0,0,7,7,7]
                ],
                "fproc_par_sched": [
                        [0,0,0,0,0,0,0],
                        [0,0,0,0,0,0,0],
                        [1,1,0,0,0,0,0],
                        [2,2,2,0,0,0,0],
                        [3,3,3,3,0,0,0],
                        [1,0,0,0,1,0,0],
                        [5,0,0,0,5,5,0],
                        [6,0,0,0,6,6,6]
                ],
                "fproc_derv_sched": [ 
                        [1,2,3,4,5,6,7],
                        [1,2,3,4,5,6,7],
                        [1,2,3,4,5,6,7],
                        [1,2,3,4,5,6,7],
                        [1,2,3,4,5,6,7],
                        [1,2,3,4,5,6,7],
                        [1,2,3,4,5,6,7],
                        [1,2,3,4,5,6,7]
                ]
            }
        elif self.robot.get_num_joints() == 7 and len(self.get_branch_parent_lids()) == 1 and self.num_PEs == 4:
            # iiwa-pants on 4 PEs
            sched_table_dict = {
                "len_fproc_sched": 11,
                "rnea_fproc_curr_sched": [1,2,3,4,5,6,7,0,0,0,0], 
                "rnea_fproc_par_sched":  [0,1,2,3,1,5,6,7,0,0,0], 
                "fproc_curr_sched": [
                        [0,0,0,0],
                        [1,0,0,0],
                        [2,2,0,0],
                        [3,3,3,0],
                        [4,4,4,4],
                        [5,0,0,0],
                        [6,0,0,0],
                        [7,0,0,0],
                        [5,0,0,0],
                        [6,6,0,0],
                        [7,7,7,0]
                ],
                "fproc_par_sched": [
                        [0,0,0,0],
                        [0,0,0,0],
                        [1,1,0,0],
                        [2,2,2,0],
                        [3,3,3,3],
                        [1,0,0,0],
                        [5,0,0,0],
                        [6,0,0,0],
                        [1,0,0,0],
                        [5,5,0,0],
                        [6,6,6,0]
                ],
                "fproc_derv_sched": [ 
                        [1,2,3,4],
                        [1,2,3,4],
                        [1,2,3,4],
                        [1,2,3,4],
                        [1,2,3,4],
                        [1,2,3,4],
                        [1,2,3,4],
                        [1,2,3,4],
                        [5,6,7,4],
                        [5,6,7,4],
                        [5,6,7,4]
                ]
            }
        elif self.robot.get_num_joints() == 12 and self.num_PEs == 7:
            # hyq on 7 PEs
            sched_table_dict = {
                "len_fproc_sched": 13,
                "rnea_fproc_curr_sched": [1,2,3,4,5,6,7,8,9,10,11,12,0], 
                "rnea_fproc_par_sched":  [0,1,2,0,4,5,0,7,8, 0,10,11,0], 
                "fproc_curr_sched": [
                        [ 0, 0, 0, 0, 0, 0, 0],
                        [ 1, 0, 0, 0, 0, 0, 0],
                        [ 2, 2, 0, 0, 0, 0, 0],
                        [ 3, 3, 3, 0, 0, 0, 0],
                        [ 4, 0, 0, 0, 0, 0, 0],
                        [ 5, 5, 0, 0, 0, 0, 0],
                        [ 6, 6, 6, 0, 0, 0, 0],
                        [ 7, 0, 0, 0, 0, 0, 0],
                        [ 8, 8, 0, 0, 0, 0, 0],
                        [ 9, 9, 9, 0, 0, 0, 0],
                        [10, 0, 0, 0, 0, 0, 0],
                        [11,11, 0, 0, 0, 0, 0],
                        [12,12,12, 0, 0, 0, 0],
                ],
                "fproc_par_sched": [
                        [ 0, 0, 0, 0, 0, 0, 0],
                        [ 0, 0, 0, 0, 0, 0, 0],
                        [ 1, 1, 0, 0, 0, 0, 0],
                        [ 2, 2, 2, 0, 0, 0, 0],
                        [ 0, 0, 0, 0, 0, 0, 0],
                        [ 4, 4, 0, 0, 0, 0, 0],
                        [ 5, 5, 5, 0, 0, 0, 0],
                        [ 0, 0, 0, 0, 0, 0, 0],
                        [ 7, 7, 0, 0, 0, 0, 0],
                        [ 8, 8, 8, 0, 0, 0, 0],
                        [ 0, 0, 0, 0, 0, 0, 0],
                        [10,10, 0, 0, 0, 0, 0],
                        [11,11,11, 0, 0, 0, 0],
                ],
                "fproc_derv_sched": [ 
                        [ 1, 2, 3, 1, 1, 1, 1],
                        [ 1, 2, 3, 1, 1, 1, 1],
                        [ 1, 2, 3, 1, 1, 1, 1],
                        [ 1, 2, 3, 1, 1, 1, 1],
                        [ 4, 5, 6, 1, 1, 1, 1],
                        [ 4, 5, 6, 1, 1, 1, 1],
                        [ 4, 5, 6, 1, 1, 1, 1],
                        [ 7, 8, 9, 1, 1, 1, 1],
                        [ 7, 8, 9, 1, 1, 1, 1],
                        [ 7, 8, 9, 1, 1, 1, 1],
                        [10,11,12, 1, 1, 1, 1],
                        [10,11,12, 1, 1, 1, 1],
                        [10,11,12, 1, 1, 1, 1],
                ]
            }
        elif self.robot.get_num_joints() == 15 and self.num_PEs == 7:
            # baxter on 7 PEs
            sched_table_dict = {
                "len_fproc_sched": 16,
                "rnea_fproc_curr_sched": [2,3,4,5,6,7,8,9,10,11,12,13,14,15,1,0], 
                "rnea_fproc_par_sched":  [0,2,3,4,5,6,7,0, 9,10,11,12,13,14,0,0], 
                "fproc_curr_sched": [
                        [ 0, 0, 0, 0, 0, 0, 0],
                        [ 2, 0, 0, 0, 0, 0, 0],
                        [ 3, 3, 0, 0, 0, 0, 0],
                        [ 4, 4, 4, 0, 0, 0, 0],
                        [ 5, 5, 5, 5, 0, 0, 0],
                        [ 6, 6, 6, 6, 6, 0, 0],
                        [ 7, 7, 7, 7, 7, 7, 0],
                        [ 8, 8, 8, 8, 8, 8, 8],
                        [ 9, 0, 0, 0, 0, 0, 0],
                        [10,10, 0, 0, 0, 0, 0],
                        [11,11,11, 0, 0, 0, 0],
                        [12,12,12,12, 0, 0, 0],
                        [13,13,13,13,13, 0, 0],
                        [14,14,14,14,14,14, 0],
                        [15,15,15,15,15,15,15],
                        [ 1, 0, 0, 0, 0, 0, 0],
                ],
                "fproc_par_sched": [
                        [ 0, 0, 0, 0, 0, 0, 0],
                        [ 0, 0, 0, 0, 0, 0, 0],
                        [ 2, 2, 0, 0, 0, 0, 0],
                        [ 3, 3, 3, 0, 0, 0, 0],
                        [ 4, 4, 4, 4, 0, 0, 0],
                        [ 5, 5, 5, 5, 5, 0, 0],
                        [ 6, 6, 6, 6, 6, 6, 0],
                        [ 7, 7, 7, 7, 7, 7, 7],
                        [ 0, 0, 0, 0, 0, 0, 0],
                        [ 9, 9, 0, 0, 0, 0, 0],
                        [10,10,10, 0, 0, 0, 0],
                        [11,11,11,11, 0, 0, 0],
                        [12,12,12,12,12, 0, 0],
                        [13,13,13,13,13,13, 0],
                        [14,14,14,14,14,14,14],
                        [ 0, 0, 0, 0, 0, 0, 0],
                ],
                "fproc_derv_sched": [ 
                        [ 2, 3, 4, 5, 6, 7, 8],
                        [ 2, 3, 4, 5, 6, 7, 8],
                        [ 2, 3, 4, 5, 6, 7, 8],
                        [ 2, 3, 4, 5, 6, 7, 8],
                        [ 2, 3, 4, 5, 6, 7, 8],
                        [ 2, 3, 4, 5, 6, 7, 8],
                        [ 2, 3, 4, 5, 6, 7, 8],
                        [ 2, 3, 4, 5, 6, 7, 8],
                        [ 9,10,11,12,13,14,15],
                        [ 9,10,11,12,13,14,15],
                        [ 9,10,11,12,13,14,15],
                        [ 9,10,11,12,13,14,15],
                        [ 9,10,11,12,13,14,15],
                        [ 9,10,11,12,13,14,15],
                        [ 9,10,11,12,13,14,15],
                        [ 1,10,11,12,13,14,15],
                ]
            }
        elif self.robot.get_num_joints() == 15 and self.num_PEs == 6:
            # baxter on 6 PEs
            sched_table_dict = {
                "len_fproc_sched": 16,
                "rnea_fproc_curr_sched": [2,3,4,5,6,7,8,9,10,11,12,13,14,15,1,0], 
                "rnea_fproc_par_sched":  [0,2,3,4,5,6,7,0, 9,10,11,12,13,14,0,0], 
                "fproc_curr_sched": [
                        [ 0, 0, 0, 0, 0, 0],
                        [ 2, 0, 0, 0, 0, 0],
                        [ 3, 3, 0, 0, 0, 0],
                        [ 4, 4, 4, 0, 0, 0],
                        [ 5, 5, 5, 5, 0, 0],
                        [ 6, 6, 6, 6, 6, 0],
                        [ 7, 7, 7, 7, 7, 7],
                        [ 8, 8, 8, 8, 8, 8],
                        [ 9, 8, 0, 0, 0, 0],
                        [10,10, 0, 0, 0, 0],
                        [11,11,11, 0, 0, 0],
                        [12,12,12,12, 0, 0],
                        [13,13,13,13,13, 0],
                        [14,14,14,14,14,14],
                        [15,15,15,15,15,15],
                        [ 1,15, 0, 0, 0, 0],
                ],
                "fproc_par_sched": [
                        [ 0, 0, 0, 0, 0, 0],
                        [ 0, 0, 0, 0, 0, 0],
                        [ 2, 2, 0, 0, 0, 0],
                        [ 3, 3, 3, 0, 0, 0],
                        [ 4, 4, 4, 4, 0, 0],
                        [ 5, 5, 5, 5, 5, 0],
                        [ 6, 6, 6, 6, 6, 6],
                        [ 7, 7, 7, 7, 7, 7],
                        [ 0, 7, 0, 0, 0, 0],
                        [ 9, 9, 0, 0, 0, 0],
                        [10,10,10, 0, 0, 0],
                        [11,11,11,11, 0, 0],
                        [12,12,12,12,12, 0],
                        [13,13,13,13,13,13],
                        [14,14,14,14,14,14],
                        [ 0,14, 0, 0, 0, 0],
                ],
                "fproc_derv_sched": [ 
                        [ 2, 3, 4, 5, 6, 7],
                        [ 2, 3, 4, 5, 6, 7],
                        [ 2, 3, 4, 5, 6, 7],
                        [ 2, 3, 4, 5, 6, 7],
                        [ 2, 3, 4, 5, 6, 7],
                        [ 2, 3, 4, 5, 6, 7],
                        [ 2, 3, 4, 5, 6, 7],
                        [ 2, 3, 4, 5, 6, 7],
                        [ 9, 8,11,12,13,14],
                        [ 9,10,11,12,13,14],
                        [ 9,10,11,12,13,14],
                        [ 9,10,11,12,13,14],
                        [ 9,10,11,12,13,14],
                        [ 9,10,11,12,13,14],
                        [ 9,10,11,12,13,14],
                        [ 1,15,11,12,13,14],
                ]
            }
        elif self.robot.get_num_joints() == 15 and self.num_PEs == 4:
            # baxter on 4 PEs
            sched_table_dict = {
                "len_fproc_sched": 18,
                "rnea_fproc_curr_sched": [2,3,4,5,6,7,8,9,10,11,12,13,14,15,1,0,0,0], 
                "rnea_fproc_par_sched":  [0,2,3,4,5,6,7,0, 9,10,11,12,13,14,0,0,0,0], 
                "fproc_curr_sched": [
                        [ 0, 0, 0, 0],
                        [ 2, 0, 0, 0],
                        [ 3, 3, 0, 0],
                        [ 4, 4, 4, 0],
                        [ 5, 5, 5, 5],
                        [ 6, 6, 6, 6],
                        [ 7, 7, 7, 7],
                        [ 8, 8, 8, 8],
                        [ 9, 8, 7, 6],
                        [10,10, 8, 7],
                        [11,11,11, 8],
                        [12,12,12,12],
                        [13,13,13,13],
                        [14,14,14,14],
                        [15,15,15,15],
                        [ 1,15,14,13],
                        [ 0, 0,15,14],
                        [ 0, 0, 0,15],
                ],
                "fproc_par_sched": [
                        [ 0, 0, 0, 0],
                        [ 0, 0, 0, 0],
                        [ 2, 2, 0, 0],
                        [ 3, 3, 3, 0],
                        [ 4, 4, 4, 4],
                        [ 5, 5, 5, 5],
                        [ 6, 6, 6, 6],
                        [ 7, 7, 7, 7],
                        [ 0, 7, 6, 5],
                        [ 9, 9, 7, 6],
                        [10,10,10, 7],
                        [11,11,11,11],
                        [12,12,12,12],
                        [13,13,13,13],
                        [14,14,14,14],
                        [ 0,14,13,12],
                        [ 0, 0,14,13],
                        [ 0, 0, 0,14],
                ],
                "fproc_derv_sched": [ 
                        [ 2, 3, 4, 5],
                        [ 2, 3, 4, 5],
                        [ 2, 3, 4, 5],
                        [ 2, 3, 4, 5],
                        [ 2, 3, 4, 5],
                        [ 2, 3, 4, 5],
                        [ 2, 3, 4, 5],
                        [ 2, 3, 4, 5],
                        [ 9, 8, 7, 6],
                        [ 9,10, 7, 6],
                        [ 9,10,11, 6],
                        [ 9,10,11,12],
                        [ 9,10,11,12],
                        [ 9,10,11,12],
                        [ 9,10,11,12],
                        [ 1,15,14,13],
                        [ 1,15,14,13],
                        [ 1,15,14,13],
                ]
            }
        elif self.robot.get_num_joints() == 30 and self.num_PEs == 7: #len(self.get_branch_parent_lids()) == 2 and self.num_PEs == 7:
            # atlas-30 on 7 PEs
            sched_table_dict = {
                "len_fproc_sched": 31,
                "rnea_fproc_curr_sched": [ 1, 2, 3, 4, 5, 6, 7, 8, 9,10,12,13,14,15,16,17,18,11,19,20,21,22,23,24,25,26,27,28,29,30, 0], 
                "rnea_fproc_par_sched":  [ 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 3,12,13,14,15,16,17, 3, 0,19,20,21,22,23, 0,25,26,27,28,29, 0], 
                "fproc_curr_sched": [
                        [ 0,  0,  0,  0,  0,  0,  0],
                        [ 1,  0,  0,  0,  0,  0,  0],
                        [ 2,  2,  0,  0,  0,  0,  0],
                        [ 3,  3,  3,  0,  0,  0,  0],
                        [ 4,  4,  4,  4,  0,  0,  0],
                        [ 5,  5,  5,  5,  5,  0,  0],
                        [ 6,  6,  6,  6,  6,  6,  0],
                        [ 7,  7,  7,  7,  7,  7,  7],
                        [ 8,  8,  8,  8,  8,  8,  8],
                        [ 9,  9,  9,  9,  9,  9,  9],
                        [10, 10, 10, 10, 10, 10, 10],
                        [12, 12, 12,  8,  9, 10, 12],
                        [13, 13, 13,  9, 10, 13, 13],
                        [14, 14, 14, 10, 14, 14, 14],
                        [15, 15, 15, 15, 15, 15, 15],
                        [16, 16, 16, 16, 16, 16, 16],
                        [17, 17, 17, 17, 17, 17, 17],
                        [18, 18, 18, 18, 18, 18, 18],
                        [11, 11, 11, 16, 17, 18, 11],
                        [19,  0,  0, 17, 18,  0,  0],
                        [20, 20,  0, 18,  0,  0,  0],
                        [21, 21, 21,  0,  0,  0,  0],
                        [22, 22, 22, 22,  0,  0,  0],
                        [23, 23, 23, 23, 23,  0,  0],
                        [24, 24, 24, 24, 24, 24,  0],
                        [25,  0,  0,  0,  0,  0,  0],
                        [26, 26,  0,  0,  0,  0,  0],
                        [27, 27, 27,  0,  0,  0,  0],
                        [28, 28, 28, 28,  0,  0,  0],
                        [29, 29, 29, 29, 29,  0,  0],
                        [30, 30, 30, 30, 30, 30,  0]
                ],
                "fproc_par_sched":  [
                        [ 0,  0,  0,  0,  0,  0,  0],
                        [ 0,  0,  0,  0,  0,  0,  0],
                        [ 1,  1,  0,  0,  0,  0,  0],
                        [ 2,  2,  2,  0,  0,  0,  0],
                        [ 3,  3,  3,  3,  0,  0,  0],
                        [ 4,  4,  4,  4,  4,  0,  0],
                        [ 5,  5,  5,  5,  5,  5,  0],
                        [ 6,  6,  6,  6,  6,  6,  6],
                        [ 7,  7,  7,  7,  7,  7,  7],
                        [ 8,  8,  8,  8,  8,  8,  8],
                        [ 9,  9,  9,  9,  9,  9,  9],
                        [ 3,  3,  3,  7,  8,  9,  3],
                        [12, 12, 12,  8,  9, 12, 12],
                        [13, 13, 13,  9, 13, 13, 13],
                        [14, 14, 14, 14, 14, 14, 14],
                        [15, 15, 15, 15, 15, 15, 15],
                        [16, 16, 16, 16, 16, 16, 16],
                        [17, 17, 17, 17, 17, 17, 17],
                        [ 3,  3,  3, 15, 16, 17,  3],
                        [ 0,  0,  0, 16, 17,  0,  0],
                        [19, 19,  0, 17,  0,  0,  0],
                        [20, 20, 20,  0,  0,  0,  0],
                        [21, 21, 21, 21,  0,  0,  0],
                        [22, 22, 22, 22, 22,  0,  0],
                        [23, 23, 23, 23, 23, 23,  0],
                        [ 0,  0,  0,  0,  0,  0,  0],
                        [25, 25,  0,  0,  0,  0,  0],
                        [26, 26, 26,  0,  0,  0,  0],
                        [27, 27, 27, 27,  0,  0,  0],
                        [28, 28, 28, 28, 28,  0,  0],
                        [29, 29, 29, 29, 29, 29,  0]
                ],
                "fproc_derv_sched": [
                        [ 1,  2,  3,  4,  5,  6,  7],
                        [ 1,  2,  3,  4,  5,  6,  7],
                        [ 1,  2,  3,  4,  5,  6,  7],
                        [ 1,  2,  3,  4,  5,  6,  7],
                        [ 1,  2,  3,  4,  5,  6,  7],
                        [ 1,  2,  3,  4,  5,  6,  7],
                        [ 1,  2,  3,  4,  5,  6,  7],
                        [ 1,  2,  3,  4,  5,  6,  7],
                        [ 1,  2,  3,  4,  5,  6,  7],
                        [ 1,  2,  3,  4,  5,  6,  7],
                        [ 1,  2,  3,  4,  5,  6,  7],
                        [ 1,  2,  3,  8,  9, 10, 12],
                        [ 1,  2,  3,  8,  9, 13, 12],
                        [ 1,  2,  3,  8, 14, 13, 12],
                        [ 1,  2,  3, 15, 14, 13, 12],
                        [ 1,  2,  3, 15, 14, 13, 12],
                        [ 1,  2,  3, 15, 14, 13, 12],
                        [ 1,  2,  3, 15, 14, 13, 12],
                        [ 1,  2,  3, 16, 17, 18, 11],
                        [19, 20, 21, 16, 17, 24, 11],
                        [19, 20, 21, 16, 23, 24, 11],
                        [19, 20, 21, 22, 23, 24, 11],
                        [19, 20, 21, 22, 23, 24, 11],
                        [19, 20, 21, 22, 23, 24, 11],
                        [19, 20, 21, 22, 23, 24, 11],
                        [25, 26, 27, 28, 29, 30, 11],
                        [25, 26, 27, 28, 29, 30, 11],
                        [25, 26, 27, 28, 29, 30, 11],
                        [25, 26, 27, 28, 29, 30, 11],
                        [25, 26, 27, 28, 29, 30, 11],
                        [25, 26, 27, 28, 29, 30, 11]
                ]
            }

        return sched_table_dict

    def get_bproc_schedule_tables(self):
        #schedules_PE = self.get_bproc_schedule_tables_PE()
        #schedules_rnea = self.get_bproc_schedule_tables_rnea()

        # temporary for hard-coded schedule tables
        sched_table_dict = None

        if self.robot.get_num_joints() == 7 and len(self.get_branch_parent_lids()) == 0 and self.num_PEs == 7:
            # iiwa-7
            sched_table_dict = {
                "len_bproc_sched": 8,
                "rnea_bproc_curr_sched": [8,7,6,5,4,3,2,1], 
                "rnea_bproc_par_sched":  [7,6,5,4,3,2,1,0], 
                "bproc_curr_sched": [
                        [8,8,8,8,8,8,8],
                        [7,7,7,7,7,7,7],
                        [6,6,6,6,6,6,6],
                        [5,5,5,5,5,5,5],
                        [4,4,4,4,4,4,4],
                        [3,3,3,3,3,3,3],
                        [2,2,2,2,2,2,2],
                        [1,1,1,1,1,1,1]
                ],
                "bproc_par_sched": [
                        [7,7,7,7,7,7,7],
                        [6,6,6,6,6,6,6],
                        [5,5,5,5,5,5,5],
                        [4,4,4,4,4,4,4],
                        [3,3,3,3,3,3,3],
                        [2,2,2,2,2,2,2],
                        [1,1,1,1,1,1,1],
                        [0,0,0,0,0,0,0]
                ],
                "bproc_derv_sched": [ 
                        [1,2,3,4,5,6,7],
                        [1,2,3,4,5,6,7],
                        [1,2,3,4,5,6,7],
                        [1,2,3,4,5,6,7],
                        [1,2,3,4,5,6,7],
                        [1,2,3,4,5,6,7],
                        [1,2,3,4,5,6,7],
                        [1,2,3,4,5,6,7]
                ]
            }
        elif self.robot.get_num_joints() == 7 and len(self.get_branch_parent_lids()) == 1 and self.num_PEs == 7:
            # iiwa-pants
            sched_table_dict = {
                "len_bproc_sched": 9,
                "rnea_bproc_curr_sched": [8,7,6,5,8,4,3,2,1], 
                "rnea_bproc_par_sched":  [7,6,5,1,4,3,2,1,0], 
                "bproc_curr_sched": [
                        [8,0,0,0,8,8,8],
                        [7,0,0,0,7,7,7],
                        [6,0,0,0,6,6,6],
                        [5,0,0,0,5,5,5],
                        [8,8,8,8,0,0,0],
                        [4,4,4,4,0,0,0],
                        [3,3,3,3,0,0,0],
                        [2,2,2,2,0,0,0],
                        [1,1,1,1,1,1,1]
                ],
                "bproc_par_sched": [
                        [7,0,0,0,7,7,7],
                        [6,0,0,0,6,6,6],
                        [5,0,0,0,5,5,5],
                        [1,0,0,0,1,1,1],
                        [4,4,4,4,0,0,0],
                        [3,3,3,3,0,0,0],
                        [2,2,2,2,0,0,0],
                        [1,1,1,1,0,0,0],
                        [0,0,0,0,0,0,0]
                ],
                "bproc_derv_sched": [ 
                        [1,2,3,4,5,6,7],
                        [1,2,3,4,5,6,7],
                        [1,2,3,4,5,6,7],
                        [1,2,3,4,5,6,7],
                        [1,2,3,4,5,6,7],
                        [1,2,3,4,5,6,7],
                        [1,2,3,4,5,6,7],
                        [1,2,3,4,5,6,7],
                        [1,2,3,4,5,6,7]
                ]
            }
        elif self.robot.get_num_joints() == 7 and len(self.get_branch_parent_lids()) == 1 and self.num_PEs == 4:
            # iiwa-pants on 4 PEs
            sched_table_dict = {
                "len_bproc_sched": 18,
                "rnea_bproc_curr_sched": [8,7,6,5,8,4,3,2,1], 
                "rnea_bproc_par_sched":  [7,6,5,1,4,3,2,1,0], 
                "bproc_curr_sched": [
                        [8,0,0,0],
                        [7,0,0,0],
                        [6,0,0,0],
                        [5,0,0,0],
                        [8,8,8,8],
                        [4,4,4,4],
                        [3,3,3,3],
                        [2,2,2,2],
                        [1,1,1,1],
                        [8,8,8,0],
                        [7,7,7,0],
                        [6,6,6,0],
                        [5,5,5,0],
                        [0,0,0,0],
                        [0,0,0,0],
                        [0,0,0,0],
                        [0,0,0,0],
                        [1,1,1,0]
                ],
                "bproc_par_sched": [
                        [7,0,0,0],
                        [6,0,0,0],
                        [5,0,0,0],
                        [1,0,0,0],
                        [4,4,4,4],
                        [3,3,3,3],
                        [2,2,2,2],
                        [1,1,1,1],
                        [0,0,0,0],
                        [7,7,7,0],
                        [6,6,6,0],
                        [5,5,5,0],
                        [1,1,1,0],
                        [0,0,0,0],
                        [0,0,0,0],
                        [0,0,0,0],
                        [0,0,0,0],
                        [0,0,0,0]
                ],
                "bproc_derv_sched": [ 
                        [1,2,3,4],
                        [1,2,3,4],
                        [1,2,3,4],
                        [1,2,3,4],
                        [1,2,3,4],
                        [1,2,3,4],
                        [1,2,3,4],
                        [1,2,3,4],
                        [1,2,3,4],
                        [5,6,7,4],
                        [5,6,7,4],
                        [5,6,7,4],
                        [5,6,7,4],
                        [5,6,7,4],
                        [5,6,7,4],
                        [5,6,7,4],
                        [5,6,7,4],
                        [5,6,7,4]
                ]
            }
        elif self.robot.get_num_joints() == 12 and self.num_PEs == 7:
            # hyq on 7 PEs
            sched_table_dict = {
                "len_bproc_sched": 16,
                "rnea_bproc_curr_sched": [13,12,11,10,13,9,8,7,13,6,5,4,13,3,2,1],
                "rnea_bproc_par_sched":  [12,11,10, 0, 9,8,7,0, 6,5,4,0, 3,2,1,0],
                "bproc_curr_sched": [
                        [13,13,13, 0, 0, 0, 0],
                        [12,12,12, 0, 0, 0, 0],
                        [11,11,11, 0, 0, 0, 0],
                        [10,10,10, 0, 0, 0, 0],
                        [13,13,13, 0, 0, 0, 0],
                        [ 9, 9, 9, 0, 0, 0, 0],
                        [ 8, 8, 8, 0, 0, 0, 0],
                        [ 7, 7, 7, 0, 0, 0, 0],
                        [13,13,13, 0, 0, 0, 0],
                        [ 6, 6, 6, 0, 0, 0, 0],
                        [ 5, 5, 5, 0, 0, 0, 0],
                        [ 4, 4, 4, 0, 0, 0, 0],
                        [13,13,13, 0, 0, 0, 0],
                        [ 3, 3, 3, 0, 0, 0, 0],
                        [ 2, 2, 2, 0, 0, 0, 0],
                        [ 1, 1, 1, 0, 0, 0, 0],
                ],
                "bproc_par_sched": [
                        [12,12,12, 0, 0, 0, 0],
                        [11,11,11, 0, 0, 0, 0],
                        [10,10,10, 0, 0, 0, 0],
                        [ 0, 0, 0, 0, 0, 0, 0],
                        [ 9, 9, 9, 0, 0, 0, 0],
                        [ 8, 8, 8, 0, 0, 0, 0],
                        [ 7, 7, 7, 0, 0, 0, 0],
                        [ 0, 0, 0, 0, 0, 0, 0],
                        [ 6, 6, 6, 0, 0, 0, 0],
                        [ 5, 5, 5, 0, 0, 0, 0],
                        [ 4, 4, 4, 0, 0, 0, 0],
                        [ 0, 0, 0, 0, 0, 0, 0],
                        [ 3, 3, 3, 0, 0, 0, 0],
                        [ 2, 2, 2, 0, 0, 0, 0],
                        [ 1, 1, 1, 0, 0, 0, 0],
                        [ 0, 0, 0, 0, 0, 0, 0],
                ],
                "bproc_derv_sched": [ 
                        [10,11,12, 1, 1, 1, 1],
                        [10,11,12, 1, 1, 1, 1],
                        [10,11,12, 1, 1, 1, 1],
                        [10,11,12, 1, 1, 1, 1],
                        [ 7, 8, 9, 1, 1, 1, 1],
                        [ 7, 8, 9, 1, 1, 1, 1],
                        [ 7, 8, 9, 1, 1, 1, 1],
                        [ 7, 8, 9, 1, 1, 1, 1],
                        [ 4, 5, 6, 1, 1, 1, 1],
                        [ 4, 5, 6, 1, 1, 1, 1],
                        [ 4, 5, 6, 1, 1, 1, 1],
                        [ 4, 5, 6, 1, 1, 1, 1],
                        [ 1, 2, 3, 1, 1, 1, 1],
                        [ 1, 2, 3, 1, 1, 1, 1],
                        [ 1, 2, 3, 1, 1, 1, 1],
                        [ 1, 2, 3, 1, 1, 1, 1],
                ]
            }
        elif self.robot.get_num_joints() == 15 and self.num_PEs == 7:
            # baxter on 7 PEs
            sched_table_dict = {
                "len_bproc_sched": 18,
                "rnea_bproc_curr_sched": [16,15,14,13,12,11,10,9,16,8,7,6,5,4,3,2,16,1], 
                "rnea_bproc_par_sched":  [15,14,13,12,11,10, 9,0, 8,7,6,5,4,3,2,0, 1,0], 
                "bproc_curr_sched": [
                        [16,16,16,16,16,16,16],
                        [15,15,15,15,15,15,15],
                        [14,14,14,14,14,14,14],
                        [13,13,13,13,13,13,13],
                        [12,12,12,12,12,12,12],
                        [11,11,11,11,11,11,11],
                        [10,10,10,10,10,10,10],
                        [ 9, 9, 9, 9, 9, 9, 9],
                        [16,16,16,16,16,16,16],
                        [ 8, 8, 8, 8, 8, 8, 8],
                        [ 7, 7, 7, 7, 7, 7, 7],
                        [ 6, 6, 6, 6, 6, 6, 6],
                        [ 5, 5, 5, 5, 5, 5, 5],
                        [ 4, 4, 4, 4, 4, 4, 4],
                        [ 3, 3, 3, 3, 3, 3, 3],
                        [ 2, 2, 2, 2, 2, 2, 2],
                        [16,16,16,16,16,16,16],
                        [ 1, 1, 1, 1, 1, 1, 1],
                ],
                "bproc_par_sched": [
                        [15,15,15,15,15,15,15],
                        [14,14,14,14,14,14,14],
                        [13,13,13,13,13,13,13],
                        [12,12,12,12,12,12,12],
                        [11,11,11,11,11,11,11],
                        [10,10,10,10,10,10,10],
                        [ 9, 9, 9, 9, 9, 9, 9],
                        [ 0, 0, 0, 0, 0, 0, 0],
                        [ 8, 8, 8, 8, 8, 8, 8],
                        [ 7, 7, 7, 7, 7, 7, 7],
                        [ 6, 6, 6, 6, 6, 6, 6],
                        [ 5, 5, 5, 5, 5, 5, 5],
                        [ 4, 4, 4, 4, 4, 4, 4],
                        [ 3, 3, 3, 3, 3, 3, 3],
                        [ 2, 2, 2, 2, 2, 2, 2],
                        [ 0, 0, 0, 0, 0, 0, 0],
                        [ 1, 1, 1, 1, 1, 1, 1],
                        [ 0, 0, 0, 0, 0, 0, 0],
                ],
                "bproc_derv_sched": [ 
                        [ 9,10,11,12,13,14,15],
                        [ 9,10,11,12,13,14,15],
                        [ 9,10,11,12,13,14,15],
                        [ 9,10,11,12,13,14,15],
                        [ 9,10,11,12,13,14,15],
                        [ 9,10,11,12,13,14,15],
                        [ 9,10,11,12,13,14,15],
                        [ 9,10,11,12,13,14,15],
                        [ 2, 3, 4, 5, 6, 7, 8],
                        [ 2, 3, 4, 5, 6, 7, 8],
                        [ 2, 3, 4, 5, 6, 7, 8],
                        [ 2, 3, 4, 5, 6, 7, 8],
                        [ 2, 3, 4, 5, 6, 7, 8],
                        [ 2, 3, 4, 5, 6, 7, 8],
                        [ 2, 3, 4, 5, 6, 7, 8],
                        [ 2, 3, 4, 5, 6, 7, 8],
                        [ 1, 3, 4, 5, 6, 7, 8],
                        [ 1, 3, 4, 5, 6, 7, 8],
                ]
            }
        elif self.robot.get_num_joints() == 15 and self.num_PEs == 6:
            # baxter on 6 PEs
            sched_table_dict = {
                "len_bproc_sched": 24,
                "rnea_bproc_curr_sched": [16,15,14,13,12,11,10,9,16,8,7,6,5,4,3,2,16,1,0,0,0,0,0,0], 
                "rnea_bproc_par_sched":  [15,14,13,12,11,10, 9,0, 8,7,6,5,4,3,2,0, 1,0,0,0,0,0,0,0], 
                "bproc_curr_sched": [
                        [16,16,16,16,16,16],
                        [15,15,15,15,15,15],
                        [14,14,14,14,14,14],
                        [13,13,13,13,13,13],
                        [12,12,12,12,12,12],
                        [11,11,11,11,11,11],
                        [10,10,10,10,10,10],
                        [ 9, 9, 9, 9, 9, 9],
                        [16,16,16,16,16,16],
                        [ 8, 8, 8, 8, 8, 8],
                        [ 7, 7, 7, 7, 7, 7],
                        [ 6, 6, 6, 6, 6, 6],
                        [ 5, 5, 5, 5, 5, 5],
                        [ 4, 4, 4, 4, 4, 4],
                        [ 3, 3, 3, 3, 3, 3],
                        [ 2, 2, 2, 2, 2, 2],
                        [16,16,16, 0, 0, 0],
                        [ 1,15, 8, 0, 0, 0],
                        [ 0,14, 7, 0, 0, 0],
                        [ 0,13, 6, 0, 0, 0],
                        [ 0,12, 5, 0, 0, 0],
                        [ 0,11, 4, 0, 0, 0],
                        [ 0,10, 3, 0, 0, 0],
                        [ 0, 9, 2, 0, 0, 0],
                ],
                "bproc_par_sched": [
                        [15,15,15,15,15,15],
                        [14,14,14,14,14,14],
                        [13,13,13,13,13,13],
                        [12,12,12,12,12,12],
                        [11,11,11,11,11,11],
                        [10,10,10,10,10,10],
                        [ 9, 9, 9, 9, 9, 9],
                        [ 0, 0, 0, 0, 0, 0],
                        [ 8, 8, 8, 8, 8, 8],
                        [ 7, 7, 7, 7, 7, 7],
                        [ 6, 6, 6, 6, 6, 6],
                        [ 5, 5, 5, 5, 5, 5],
                        [ 4, 4, 4, 4, 4, 4],
                        [ 3, 3, 3, 3, 3, 3],
                        [ 2, 2, 2, 2, 2, 2],
                        [ 0, 0, 0, 0, 0, 0],
                        [ 1,15, 8, 0, 0, 0],
                        [ 0,14, 7, 0, 0, 0],
                        [ 0,13, 6, 0, 0, 0],
                        [ 0,12, 5, 0, 0, 0],
                        [ 0,11, 4, 0, 0, 0],
                        [ 0,10, 3, 0, 0, 0],
                        [ 0, 9, 2, 0, 0, 0],
                        [ 0, 0, 0, 0, 0, 0],
                ],
                "bproc_derv_sched": [ 
                        [ 9,10,11,12,13,14],
                        [ 9,10,11,12,13,14],
                        [ 9,10,11,12,13,14],
                        [ 9,10,11,12,13,14],
                        [ 9,10,11,12,13,14],
                        [ 9,10,11,12,13,14],
                        [ 9,10,11,12,13,14],
                        [ 9,10,11,12,13,14],
                        [ 2, 3, 4, 5, 6, 7],
                        [ 2, 3, 4, 5, 6, 7],
                        [ 2, 3, 4, 5, 6, 7],
                        [ 2, 3, 4, 5, 6, 7],
                        [ 2, 3, 4, 5, 6, 7],
                        [ 2, 3, 4, 5, 6, 7],
                        [ 2, 3, 4, 5, 6, 7],
                        [ 2, 3, 4, 5, 6, 7],
                        [ 1,15, 8, 5, 6, 7],
                        [ 1,15, 8, 5, 6, 7],
                        [ 1,15, 8, 5, 6, 7],
                        [ 1,15, 8, 5, 6, 7],
                        [ 1,15, 8, 5, 6, 7],
                        [ 1,15, 8, 5, 6, 7],
                        [ 1,15, 8, 5, 6, 7],
                        [ 1,15, 8, 5, 6, 7],
                ]
            }
        elif self.robot.get_num_joints() == 15 and self.num_PEs == 4:
            # baxter on 4 PEs
            sched_table_dict = {
                "len_bproc_sched": 32,
                "rnea_bproc_curr_sched": [16,15,14,13,12,11,10,9,16,8,7,6,5,4,3,2,16,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0], 
                "rnea_bproc_par_sched":  [15,14,13,12,11,10, 9,0, 8,7,6,5,4,3,2,0, 1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0], 
                "bproc_curr_sched": [
                        [16,16,16,16],
                        [15,15,15,15],
                        [14,14,14,14],
                        [13,13,13,13],
                        [12,12,12,12],
                        [11,11,11,11],
                        [10,10,10,10],
                        [ 9, 9, 9, 9],
                        [16,16,16,16],
                        [ 8, 8, 8, 8],
                        [ 7, 7, 7, 7],
                        [ 6, 6, 6, 6],
                        [ 5, 5, 5, 5],
                        [ 4, 4, 4, 4],
                        [ 3, 3, 3, 3],
                        [ 2, 2, 2, 2],
                        [16,16,16,16],
                        [15,15, 8,15],
                        [14,14, 7,14],
                        [13,13, 6,13],
                        [12,12, 5,12],
                        [11,11, 4,11],
                        [10,10, 3,10],
                        [ 9, 9, 2, 9],
                        [16,16,16, 0],
                        [ 8, 8, 1, 0],
                        [ 7, 7, 0, 0],
                        [ 6, 6, 0, 0],
                        [ 5, 5, 0, 0],
                        [ 4, 4, 0, 0],
                        [ 3, 3, 0, 0],
                        [ 2, 2, 0, 0],
                ],
                "bproc_par_sched": [
                        [15,15,15,15],
                        [14,14,14,14],
                        [13,13,13,13],
                        [12,12,12,12],
                        [11,11,11,11],
                        [10,10,10,10],
                        [ 9, 9, 9, 9],
                        [ 0, 0, 0, 0],
                        [ 8, 8, 8, 8],
                        [ 7, 7, 7, 7],
                        [ 6, 6, 6, 6],
                        [ 5, 5, 5, 5],
                        [ 4, 4, 4, 4],
                        [ 3, 3, 3, 3],
                        [ 2, 2, 2, 2],
                        [ 0, 0, 0, 0],
                        [15,15, 8,15],
                        [14,14, 7,14],
                        [13,13, 6,13],
                        [12,12, 5,12],
                        [11,11, 4,11],
                        [10,10, 3,10],
                        [ 9, 9, 2, 9],
                        [ 0, 0, 0, 0],
                        [ 8, 8, 1, 0],
                        [ 7, 7, 0, 0],
                        [ 6, 6, 0, 0],
                        [ 5, 5, 0, 0],
                        [ 4, 4, 0, 0],
                        [ 3, 3, 0, 0],
                        [ 2, 2, 0, 0],
                        [ 0, 0, 0, 0],
                ],
                "bproc_derv_sched": [ 
                        [ 9,10,11,15],
                        [ 9,10,11,15],
                        [ 9,10,11,15],
                        [ 9,10,11,15],
                        [ 9,10,11,15],
                        [ 9,10,11,15],
                        [ 9,10,11,15],
                        [ 9,10,11,15],
                        [ 2, 3, 4, 8],
                        [ 2, 3, 4, 8],
                        [ 2, 3, 4, 8],
                        [ 2, 3, 4, 8],
                        [ 2, 3, 4, 8],
                        [ 2, 3, 4, 8],
                        [ 2, 3, 4, 8],
                        [ 2, 3, 4, 8],
                        [12,13, 7,14],
                        [12,13, 7,14],
                        [12,13, 7,14],
                        [12,13, 7,14],
                        [12,13, 7,14],
                        [12,13, 7,14],
                        [12,13, 7,14],
                        [12,13, 7,14],
                        [ 6, 5, 1, 0],
                        [ 6, 5, 1, 0],
                        [ 6, 5, 0, 0],
                        [ 6, 5, 0, 0],
                        [ 6, 5, 0, 0],
                        [ 6, 5, 0, 0],
                        [ 6, 5, 0, 0],
                        [ 6, 5, 0, 0],
                ]
            }
        elif self.robot.get_num_joints() == 30 and self.num_PEs == 7: #len(self.get_branch_parent_lids()) == 2 and self.num_PEs == 7:
            # atlas-30 on 7 PEs
            sched_table_dict = {
                "len_bproc_sched": 53,
                "rnea_bproc_curr_sched": [31,18,17,16,15,14,13,12,31,10, 9, 8, 7, 6, 5, 4,31,11, 3, 2, 1,31,30,19,28,27,26,25,31,24,23,22,21,20,19, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], 
                "rnea_bproc_par_sched":  [18,17,16,15,14,13,12, 3,10, 9, 8, 7, 6, 5, 4, 3,11, 3, 2, 1, 0,30,19,28,27,26,25, 0,24,23,22,21,20,19, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                "bproc_curr_sched": [
                        [31, 31, 31, 31, 31, 31, 31],
                        [18, 18, 18, 18, 18, 18, 18],
                        [17, 17, 17, 17, 17, 17, 17],
                        [16, 16, 16, 16, 16, 16, 16],
                        [15, 15, 15, 15, 15, 15, 15],
                        [14, 14, 14, 14, 14, 14, 14],
                        [13, 13, 13, 13, 13, 13, 13],
                        [12, 12, 12, 12, 12, 12, 12],
                        [31, 31, 31,  0,  0,  0,  0],
                        [10, 10, 10,  0,  0,  0,  0],
                        [ 9,  9,  9,  0,  0,  0,  0],
                        [ 8,  8,  8,  0,  0,  0,  0],
                        [ 7,  7,  7,  0,  0,  0,  0],
                        [ 6,  6,  6,  0,  0,  0,  0],
                        [ 5,  5,  5,  0,  0,  0,  0],
                        [ 4,  4,  4,  0,  0,  0,  0],
                        [31, 31, 31,  0,  0,  0,  0],
                        [11, 11, 11,  0,  0,  0,  0],
                        [ 3,  3,  3,  3,  3,  3,  3],
                        [ 2,  2,  2,  2,  2,  2,  2],
                        [ 1,  1,  1,  1,  1,  1,  1],
                        [31, 31, 31, 31, 31, 31, 31],
                        [18, 18, 18, 10, 10, 10, 10],
                        [17, 17, 17,  9,  9,  9,  9],
                        [16, 16, 16,  8,  8,  8,  8],
                        [15, 15, 15,  7,  7,  7,  7],
                        [14, 14, 14,  6,  6,  6,  6],
                        [13, 13, 13,  5,  5,  5,  5],
                        [12, 12, 12,  4,  4,  4,  4],
                        [ 3,  3,  3,  3,  3,  3,  3],
                        [ 2,  2,  2,  2,  2,  2,  2],
                        [ 1,  1,  1,  1,  1,  1,  1],
                        [31, 31, 31, 31, 31, 31, 31],
                        [10, 10, 10, 30, 30, 30, 30],
                        [ 9,  9,  9, 29, 29, 29, 29],
                        [ 8,  8,  8, 28, 28, 28, 28],
                        [ 7,  7,  7, 27, 27, 27, 27],
                        [ 6,  6,  6, 26, 26, 26, 26],
                        [ 5,  5,  5, 25, 25, 25, 25],
                        [ 4,  4,  4, 31, 31, 31, 31],
                        [ 3,  3,  3, 30, 30, 24, 24],
                        [ 2,  2,  2, 29, 29, 23, 23],
                        [ 1,  1,  1, 28, 28, 22, 22],
                        [31, 31, 31, 27, 27, 21, 21],
                        [24, 24, 24, 26, 26, 20, 20],
                        [23, 23, 23, 25, 25, 19, 19],
                        [22, 22, 22, 31,  0,  0,  0],
                        [21, 21, 21, 24,  0,  0,  0],
                        [20, 20, 20, 23,  0,  0,  0],
                        [19, 19, 19, 22,  0,  0,  0],
                        [ 0,  0,  0, 21,  0,  0,  0],
                        [ 0,  0,  0, 20,  0,  0,  0],
                        [ 0,  0,  0, 19,  0,  0,  0]
                ],
                "bproc_par_sched": [
                        [18, 18, 18, 18, 18, 18, 18],
                        [17, 17, 17, 17, 17, 17, 17],
                        [16, 16, 16, 16, 16, 16, 16],
                        [15, 15, 15, 15, 15, 15, 15],
                        [14, 14, 14, 14, 14, 14, 14],
                        [13, 13, 13, 13, 13, 13, 13],
                        [12, 12, 12, 12, 12, 12, 12],
                        [ 3,  3,  3,  3,  3,  3,  3],
                        [10, 10, 10,  0,  0,  0,  0],
                        [ 9,  9,  9,  0,  0,  0,  0],
                        [ 8,  8,  8,  0,  0,  0,  0],
                        [ 7,  7,  7,  0,  0,  0,  0],
                        [ 6,  6,  6,  0,  0,  0,  0],
                        [ 5,  5,  5,  0,  0,  0,  0],
                        [ 4,  4,  4,  0,  0,  0,  0],
                        [ 3,  3,  3,  0,  0,  0,  0],
                        [11, 11, 11,  0,  0,  0,  0],
                        [ 3,  3,  3,  0,  0,  0,  0],
                        [ 2,  2,  2,  2,  2,  2,  2],
                        [ 1,  1,  1,  1,  1,  1,  1],
                        [ 0,  0,  0,  0,  0,  0,  0],
                        [18, 18, 18, 10, 10, 10, 10],
                        [17, 17, 17,  9,  9,  9,  9],
                        [16, 16, 16,  8,  8,  8,  8],
                        [15, 15, 15,  7,  7,  7,  7],
                        [14, 14, 14,  6,  6,  6,  6],
                        [13, 13, 13,  5,  5,  5,  5],
                        [12, 12, 12,  4,  4,  4,  4],
                        [ 3,  3,  3,  3,  3,  3,  3],
                        [ 2,  2,  2,  2,  2,  2,  2],
                        [ 1,  1,  1,  1,  1,  1,  1],
                        [ 0,  0,  0,  0,  0,  0,  0],
                        [10, 10, 10, 30, 30, 30, 30],
                        [ 9,  9,  9, 29, 29, 29, 29],
                        [ 8,  8,  8, 28, 28, 28, 28],
                        [ 7,  7,  7, 27, 27, 27, 27],
                        [ 6,  6,  6, 26, 26, 26, 26],
                        [ 5,  5,  5, 25, 25, 25, 25],
                        [ 4,  4,  4,  0,  0,  0,  0],
                        [ 3,  3,  3, 30, 30, 24, 24],
                        [ 2,  2,  2, 29, 29, 23, 23],
                        [ 1,  1,  1, 28, 28, 22, 22],
                        [ 0,  0,  0, 27, 27, 21, 21],
                        [24, 24, 24, 26, 26, 20, 20],
                        [23, 23, 23, 25, 25, 19, 19],
                        [22, 22, 22,  0,  0,  0,  0],
                        [21, 21, 21, 24,  0,  0,  0],
                        [20, 20, 20, 23,  0,  0,  0],
                        [19, 19, 19, 22,  0,  0,  0],
                        [ 0,  0,  0, 21,  0,  0,  0],
                        [ 0,  0,  0, 20,  0,  0,  0],
                        [ 0,  0,  0, 19,  0,  0,  0],
                        [ 0,  0,  0,  0,  0,  0,  0]
                ],
                "bproc_derv_sched": [ 
                        [ 1,  2,  3, 12, 13, 14, 15],
                        [ 1,  2,  3, 12, 13, 14, 15],
                        [ 1,  2,  3, 12, 13, 14, 15],
                        [ 1,  2,  3, 12, 13, 14, 15],
                        [ 1,  2,  3, 12, 13, 14, 15],
                        [ 1,  2,  3, 12, 13, 14, 15],
                        [ 1,  2,  3, 12, 13, 14, 15],
                        [ 1,  2,  3, 12, 13, 14, 15],
                        [ 1,  2,  3, 12, 13, 14, 15],
                        [ 1,  2,  3, 12, 13, 14, 15],
                        [ 1,  2,  3, 12, 13, 14, 15],
                        [ 1,  2,  3, 12, 13, 14, 15],
                        [ 1,  2,  3, 12, 13, 14, 15],
                        [ 1,  2,  3, 12, 13, 14, 15],
                        [ 1,  2,  3, 12, 13, 14, 15],
                        [ 1,  2,  3, 12, 13, 14, 15],
                        [ 1,  2,  3, 12, 13, 14, 15],
                        [ 1,  2,  3, 12, 13, 14, 15],
                        [ 1,  2,  3, 12, 13, 14, 15],
                        [ 1,  2,  3, 12, 13, 14, 15],
                        [ 1,  2,  3, 12, 13, 14, 15],
                        [16, 17, 18,  4,  5,  6,  7],
                        [16, 17, 18,  4,  5,  6,  7],
                        [16, 17, 18,  4,  5,  6,  7],
                        [16, 17, 18,  4,  5,  6,  7],
                        [16, 17, 18,  4,  5,  6,  7],
                        [16, 17, 18,  4,  5,  6,  7],
                        [16, 17, 18,  4,  5,  6,  7],
                        [16, 17, 18,  4,  5,  6,  7],
                        [16, 17, 18,  4,  5,  6,  7],
                        [16, 17, 18,  4,  5,  6,  7],
                        [16, 17, 18,  4,  5,  6,  7],
                        [ 8,  9, 10, 25, 26, 27, 28],
                        [ 8,  9, 10, 25, 26, 27, 28],
                        [ 8,  9, 10, 25, 26, 27, 28],
                        [ 8,  9, 10, 25, 26, 27, 28],
                        [ 8,  9, 10, 25, 26, 27, 28],
                        [ 8,  9, 10, 25, 26, 27, 28],
                        [ 8,  9, 10, 25, 26, 27, 28],
                        [ 8,  9, 10, 29, 30, 19, 20],
                        [ 8,  9, 10, 29, 30, 19, 20],
                        [ 8,  9, 10, 29, 30, 19, 20],
                        [ 8,  9, 10, 29, 30, 19, 20],
                        [21, 22, 23, 29, 30, 19, 20],
                        [21, 22, 23, 29, 30, 19, 20],
                        [21, 22, 23, 29, 30, 19, 20],
                        [21, 22, 23, 24, 30, 19, 20],
                        [21, 22, 23, 24, 30, 19, 20],
                        [21, 22, 23, 24, 30, 19, 20],
                        [21, 22, 23, 24, 30, 19, 20],
                        [21, 22, 23, 24, 30, 19, 20],
                        [21, 22, 23, 24, 30, 19, 20],
                        [21, 22, 23, 24, 30, 19, 20]
                ]
            }

        return sched_table_dict

    def get_matmul_schedule_tables(self):
        sched_table_dict = None
        if self.robot.get_num_joints() == 7 and self.num_PEs == 7 and self.block_size == 7:
            # iiwa-7 and iiwa-pants
            sched_table_dict = {
                "n_tiles": 1,
                "len_block_minv_sched_per_matrix": 1,
                "lh_tile_sched": [0], 
                "rh_tile_sched": [0], 
                "output_tile_incr_sched": [0], 
                "rh_tile_col_sched": [
                        [0,1,2,3,4,5,6]
                ],
                "lh_tile_row_start_sched": [0],
                "lh_tile_col_start_sched": [0],
                "rh_tile_row_start_sched": [0],
                "rh_tile_col_start_sched": [0],
                "output_tile_row_start_sched": [0],
                "output_tile_col_start_sched": [0],
            }
        elif self.robot.get_num_joints() == 7 and len(self.get_branch_parent_lids()) == 1 and self.num_PEs == 4 and self.block_size == 4:
            # iiwa-pants on 4 PEs
            assert False, "TODO!"
        elif self.robot.get_num_joints() == 12 and self.num_PEs == 7 and self.block_size == 6:
            # hyq on 7 PEs with block size = 6
            sched_table_dict = {
                "n_tiles": 4,
                "len_block_minv_sched_per_matrix": 2,
                "lh_tile_sched": [0,3], 
                "rh_tile_sched": [0,3], 
                "output_tile_incr_sched": [0,3], 
                "rh_tile_col_sched": [
                        [0,1,2,3,4,5,6],
                        [0,1,2,3,4,5,6],
                ],
                "lh_tile_row_start_sched": [0,6],
                "lh_tile_col_start_sched": [0,6],
                "rh_tile_row_start_sched": [0,6],
                "rh_tile_col_start_sched": [0,6],
                "output_tile_row_start_sched": [0,6],
                "output_tile_col_start_sched": [0,6],
            }
        elif self.robot.get_num_joints() == 15 and self.num_PEs == 7 and self.block_size == 8:
            # baxter on 7 PEs with block size = 8
            # todo: we can squeeze this from 4 to 3 by packing rh_tile_cols better
            sched_table_dict = {
                "n_tiles": 4,
                "len_block_minv_sched_per_matrix": 4,
                "lh_tile_sched": [0,0,3,3], 
                "rh_tile_sched": [0,0,3,3], 
                "output_tile_incr_sched": [0,0,3,3], 
                "rh_tile_col_sched": [
                        [0,1,2,3,4,5,6],
                        [7,8,8,8,8,8,8],
                        [0,1,2,3,4,5,6],
                        [7,8,8,8,8,8,8],
                ],
                "lh_tile_row_start_sched": [0,0,8,8],
                "lh_tile_col_start_sched": [0,0,8,8],
                "rh_tile_row_start_sched": [0,0,8,8],
                "rh_tile_col_start_sched": [0,0,8,8],
                "output_tile_row_start_sched": [0,0,8,8],
                "output_tile_col_start_sched": [0,0,8,8],
            }
        elif self.robot.get_num_joints() == 15 and self.num_PEs == 6 and self.block_size == 6:
            # baxter on 7 PEs with block size = 6
            sched_table_dict = {
                "n_tiles": 9,
                "len_block_minv_sched_per_matrix": 17,
                "lh_tile_sched":          [0,3,1,4,7,0,3,1,4,7,5,8,1,4,7,5,8], 
                "rh_tile_sched":          [0,0,3,3,3,1,1,4,4,4,7,7,5,5,5,8,8], 
                "output_tile_incr_sched": [0,3,0,3,6,1,4,1,4,7,4,7,2,5,8,5,8], 
                "rh_tile_col_sched": [
                        [0,1,2,3,4,5],
                        [0,1,2,3,4,5],
                        [0,1,2,3,4,5],
                        [0,1,2,3,4,5],
                        [0,1,2,3,4,5],
                        [0,1,2,3,4,5],
                        [0,1,2,3,4,5],
                        [0,1,2,3,4,5],
                        [0,1,2,3,4,5],
                        [0,1,2,3,4,5],
                        [0,1,2,3,4,5],
                        [0,1,2,3,4,5],
                        [0,1,2,3,4,5],
                        [0,1,2,3,4,5],
                        [0,1,2,3,4,5],
                        [0,1,2,3,4,5],
                        [0,1,2,3,4,5],
                ],
                "lh_tile_row_start_sched":     [0,6,0,6,12,0,6,0,6,12, 6,12, 0, 6,12, 6,12], 
                "lh_tile_col_start_sched":     [0,0,6,6, 6,0,0,6,6, 6,12,12, 6, 6, 6,12,12],
                "rh_tile_row_start_sched":     [0,0,6,6, 6,0,0,6,6, 6,12,12, 6, 6, 6,12,12],
                "rh_tile_col_start_sched":     [0,0,0,0, 0,6,6,6,6, 6, 6, 6,12,12,12,12,12],
                "output_tile_row_start_sched": [0,6,0,6,12,0,6,0,6,12, 6,12, 0, 6,12, 6,12],
                "output_tile_col_start_sched": [0,0,0,0, 0,6,6,6,6, 6, 6, 6,12,12,12,12,12],
            }
        elif self.robot.get_num_joints() == 15 and self.num_PEs == 4 and self.block_size == 4:
            # baxter on 4 PEs with block size = 4
            sched_table_dict = {
                "n_tiles": 16,
                "len_block_minv_sched_per_matrix": 16,
                "lh_tile_sched":          [0,4,1,5, 0,4,1,5, 10,14,11,15, 10,14,11,15], 
                "rh_tile_sched":          [0,0,4,4, 1,1,5,5, 10,10,14,14, 11,11,15,15], 
                "output_tile_incr_sched": [0,4,0,4, 1,5,1,5, 10,14,10,14, 11,15,11,15], 
                "rh_tile_col_sched": [
                        [0,1,2,3],
                        [0,1,2,3],
                        [0,1,2,3],
                        [0,1,2,3],
                        [0,1,2,3],
                        [0,1,2,3],
                        [0,1,2,3],
                        [0,1,2,3],
                        [0,1,2,3],
                        [0,1,2,3],
                        [0,1,2,3],
                        [0,1,2,3],
                        [0,1,2,3],
                        [0,1,2,3],
                        [0,1,2,3],
                        [0,1,2,3],
                ],
                "lh_tile_row_start_sched":     [0,4,0,4, 0,4,0,4, 8,12, 8,12,  8,12, 8,12], 
                "lh_tile_col_start_sched":     [0,0,4,4, 0,0,4,4, 8, 8,12,12,  8, 8,12,12],
                "rh_tile_row_start_sched":     [0,0,4,4, 0,0,4,4, 8, 8,12,12,  8, 8,12,12],
                "rh_tile_col_start_sched":     [0,0,0,0, 4,4,4,4, 8, 8, 8, 8, 12,12,12,12],
                "output_tile_row_start_sched": [0,4,0,4, 0,4,0,4, 8,12, 8,12,  8,12, 8,12],
                "output_tile_col_start_sched": [0,0,0,0, 4,4,4,4, 8, 8, 8, 8, 12,12,12,12],
            }
        elif self.robot.get_num_links_effective() == 30 and self.num_PEs == 7 and self.block_size == 12:
            # atlas
            sched_table_dict = {
                "n_tiles": 9,
                "len_block_minv_sched_per_matrix": 5,
                "lh_tile_sched":          [0,0,3,3,1,1,4,4,7,7,8,8, 0,0,3,3,1,1,4,4,7,7,8,8, 0,0,3,3,1,1,4,4,7,7,8,8], 
                "rh_tile_sched":          [0,0,0,0,3,3,3,3,3,3,6,6, 1,1,1,1,4,4,4,4,4,4,7,7, 2,2,2,2,5,5,5,5,5,5,8,8],
                "output_tile_incr_sched": [0,0,3,3,0,0,3,3,6,6,6,6, 1,1,4,4,1,1,4,4,7,7,7,7, 2,2,5,5,1,1,4,4,7,7,7,7],
                "rh_tile_col_sched": [
                        [ 0, 1, 2, 3, 4, 5, 6],
                        [ 7, 8, 9,10,11,12,13],
                        [ 0, 1, 2, 3, 4, 5, 6],
                        [ 7, 8, 9,10,11,12,13],
                        [ 0, 1, 2, 3, 4, 5, 6],
                        [ 7, 8, 9,10,11,12,13],
                        [ 0, 1, 2, 3, 4, 5, 6],
                        [ 7, 8, 9,10,11,12,13],
                        [ 0, 1, 2, 3, 4, 5, 6],
                        [ 7, 8, 9,10,11,12,13],
                        [ 0, 1, 2, 3, 4, 5, 6],
                        [ 7, 8, 9,10,11,12,13],
                        [ 0, 1, 2, 3, 4, 5, 6],
                        [ 7, 8, 9,10,11,12,13],
                        [ 0, 1, 2, 3, 4, 5, 6],
                        [ 7, 8, 9,10,11,12,13],
                        [ 0, 1, 2, 3, 4, 5, 6],
                        [ 7, 8, 9,10,11,12,13],
                        [ 0, 1, 2, 3, 4, 5, 6],
                        [ 7, 8, 9,10,11,12,13],
                        [ 0, 1, 2, 3, 4, 5, 6],
                        [ 7, 8, 9,10,11,12,13],
                        [ 0, 1, 2, 3, 4, 5, 6],
                        [ 7, 8, 9,10,11,12,13],
                        [ 0, 1, 2, 3, 4, 5, 6],
                        [ 7, 8, 9,10,11,12,13],
                        [ 0, 1, 2, 3, 4, 5, 6],
                        [ 7, 8, 9,10,11,12,13],
                        [ 0, 1, 2, 3, 4, 5, 6],
                        [ 7, 8, 9,10,11,12,13],
                        [ 0, 1, 2, 3, 4, 5, 6],
                        [ 7, 8, 9,10,11,12,13],
                        [ 0, 1, 2, 3, 4, 5, 6],
                        [ 7, 8, 9,10,11,12,13],
                        [ 0, 1, 2, 3, 4, 5, 6],
                        [ 7, 8, 9,10,11,12,13],
                ],
                "lh_tile_row_start_sched":     [ 0, 0,12,12, 0, 0,12,12,24,24,24,24,  0, 0,12,12, 0, 0,12,12,24,24,24,24,  0, 0,12,12, 0, 0,12,12,24,24,24,24],
                "lh_tile_col_start_sched":     [ 0, 0, 0, 0,12,12,12,12,12,12,24,24,  0, 0, 0, 0,12,12,12,12,12,12,24,24,  0, 0, 0, 0,12,12,12,12,12,12,24,24],
                "rh_tile_row_start_sched":     [ 0, 0, 0, 0,12,12,12,12,12,12,24,24,  0, 0, 0, 0,12,12,12,12,12,12,24,24,  0, 0, 0, 0,12,12,12,12,12,12,24,24],
                "rh_tile_col_start_sched":     [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 12,12,12,12,12,12,12,12,12,12,12,12, 24,24,24,24,24,24,24,24,24,24,24,24],
                "output_tile_row_start_sched": [ 0, 0,12,12, 0, 0,12,12,24,24,24,24,  0, 0,12,12, 0, 0,12,12,24,24,24,24,  0, 0,12,12, 0, 0,12,12,24,24,24,24],
                "output_tile_col_start_sched": [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 12,12,12,12,12,12,12,12,12,12,12,12, 24,24,24,24,24,24,24,24,24,24,24,24],
            }
        elif self.robot.get_num_links_effective() == 30 and self.num_PEs == 7 and self.block_size == 7:
            # atlas fake schedule to stress-test compilation
            sched_table_dict = {
                "n_tiles": 25,
                "len_block_minv_sched_per_matrix": 20,
                "lh_tile_sched":          [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0], 
                "rh_tile_sched":          [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                "output_tile_incr_sched": [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                "rh_tile_col_sched": [
                        [ 0, 1, 2, 3, 4, 5, 6],
                        [ 0, 1, 2, 3, 4, 5, 6],
                        [ 0, 1, 2, 3, 4, 5, 6],
                        [ 0, 1, 2, 3, 4, 5, 6],
                        [ 0, 1, 2, 3, 4, 5, 6],
                        [ 0, 1, 2, 3, 4, 5, 6],
                        [ 0, 1, 2, 3, 4, 5, 6],
                        [ 0, 1, 2, 3, 4, 5, 6],
                        [ 0, 1, 2, 3, 4, 5, 6],
                        [ 0, 1, 2, 3, 4, 5, 6],
                        [ 0, 1, 2, 3, 4, 5, 6],
                        [ 0, 1, 2, 3, 4, 5, 6],
                        [ 0, 1, 2, 3, 4, 5, 6],
                        [ 0, 1, 2, 3, 4, 5, 6],
                        [ 0, 1, 2, 3, 4, 5, 6],
                        [ 0, 1, 2, 3, 4, 5, 6],
                        [ 0, 1, 2, 3, 4, 5, 6],
                        [ 0, 1, 2, 3, 4, 5, 6],
                        [ 0, 1, 2, 3, 4, 5, 6],
                        [ 0, 1, 2, 3, 4, 5, 6],
                ],
                "lh_tile_row_start_sched":     [ 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                "lh_tile_col_start_sched":     [ 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                "rh_tile_row_start_sched":     [ 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                "rh_tile_col_start_sched":     [ 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                "output_tile_row_start_sched": [ 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                "output_tile_col_start_sched": [ 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
            }

        return sched_table_dict
