import numpy as np
from URDFParser import Joint
from rbdReference import rbdReference
import copy
from math import ceil
import os
import csv

class SchedTaskList:
    def __init__(self):
        self.sched_task_list = []
        self.ID_task_list = []
        self.dID_task_list = []
    
    def add(self, task):
        self.sched_task_list.append(task)
        if task.resource == "ID":
            self.ID_task_list.append(task)
        if task.resource == "dID":
            self.dID_task_list.append(task)

    def remove(self, task):
        for i in range(len(self.sched_task_list)):
            t = self.sched_task_list[i]
            if t.resource == task.resource and \
                    t.curr == task.curr and t.derv == task.derv and t.par == task.par:
                return self.sched_task_list.pop(i)

    def retrieve_task(self, task):
        for t in self.sched_task_list:
            if t.curr == task.curr and t.par == task.par and t.derv == task.derv:
                return t
        return None

    def find_ID_task(self, curr, par):
        for t in self.ID_task_list:
            if t.curr == curr and t.par == par:
                return t
        return None

    def find_dID_task(self, curr, derv):
        for t in self.dID_task_list:
            if t.curr == curr and t.derv == derv:
                return t
        return None

    def get_task_list(self):
        return self.sched_task_list

    def print_task_graph(self):
        for task in self.sched_task_list:
            task.dump()

    def is_empty(self):
        if self.sched_task_list:
            return False
        else:
            return True

    def get_task_list_ordered_by_dist_from_sink(self):
        # returns in descending order of dist_from_sink because
        # higher dist is higher priority
        return sorted(self.sched_task_list, key=lambda task: task.dist_from_sink, reverse=True)

    def extend(self, task_graph):
        for t in task_graph.get_task_list():
            self.add(t)

class Task:
    def __init__(self, resource="", curr=-1, derv=-1, par=-1):
        self.resource = resource # "ID", "dID"
        self.prereqs = SchedTaskList() # list of Tasks, only two prereqs needed for fproc because tree
        self.comes_before = SchedTaskList() # opposite of prereqs, lists nodes which are dependent on this one
        self.curr = curr
        self.derv = derv
        self.par = par
        self.dist_from_sink = -1
        self.is_scheduled = False

    def set_prereq(self, task):
        self.prereqs.add(task)

    def set_dist_from_sink(self, dist):
        self.dist_from_sink = dist

    def dump(self):
        print("---")
        print("this: curr:{c}, derv:{d}, par:{p}".format(c=self.curr, d=self.derv, p=self.par))
        print("dist_from_sink: "+str(self.dist_from_sink))
        print("prereqs:")
        for prereq in self.prereqs.get_task_list():
            print("curr:{c}, derv:{d}, par:{p}".format(c=prereq.curr, d=prereq.derv, p=prereq.par))
        print("comes_before:")
        for before in self.comes_before.get_task_list():
            print("curr:{c}, derv:{d}, par:{p}".format(c=before.curr, d=before.derv, p=before.par))

class ScheduleGen:
    def __init__(self, robot, num_fproc_PEs, num_bproc_PEs, block_size):
        self.num_fproc_PEs = num_fproc_PEs
        self.num_bproc_PEs = num_bproc_PEs
        self.block_size = block_size
        self.dim_list = ["AX","AY","AZ","LX","LY","LZ"]
        self.robot = robot

    ###
    def get_branch_parent_lids(self):
        robot = self.robot

        #print("Links (minus root link):")
        #print([l.get_name() for l in robot.get_links_ordered_by_id()][1:])
        #print([l.get_id()+1 for l in robot.get_links_ordered_by_id()][1:])

        #print("Joints")
        #print([j.get_name() for j in robot.get_joints_ordered_by_id()][1:])
        #print([j.get_id() for j in robot.get_joints_ordered_by_id()][1:])
        
        branch_parents_lids = []

        num_links = robot.get_num_links_effective()
        for lid in range(num_links):
            #print("lid: " + str(lid))
            link = robot.get_link_by_id(lid)
            children_joints = robot.get_joints_by_parent_name(link.get_name())
            #print(link.get_name())
            #print([child.get_name() for child in children_joints])
            if len(children_joints) > 1:
                branch_parents_lids.append(lid+1)

        return branch_parents_lids
    ###

    def get_parent_joint(self, joint):
        robot = self.robot

        parent_link_name = joint.get_parent()
        parent_joints = robot.get_joints_by_child_name(parent_link_name)
        if not parent_joints:
            return None
        else:
            assert len(parent_joints) == 1, \
                    "parent_link is child of multiple joints, not legal in tree"
            parent_joint = parent_joints[0]
            return parent_joint
    
    def get_children_joints_by_parent_joint(self, parent_joint):
        child_link_name = parent_joint.get_child()
        children_joints = self.robot.get_joints_by_parent_name(child_link_name)
        return children_joints

    def joint_chain_by_id(self, chain):
        jid_chain = [joint.get_id() for joint in chain]
        return jid_chain

    def get_fproc_chains(self):
        robot = self.robot
        robot_joints = robot.get_joints_ordered_by_id()

        fproc_chain_by_jid = {}

        # DFS from every joint
        for curr_chain_joint in robot_joints:
            curr_chain_jid = curr_chain_joint.get_id()
            # fproc chain is all the joints visited from curr_chain_joint in preorder traversal
            fproc_chain = []

            stack = []
            stack.append(curr_chain_joint)
            while stack:
                top_joint = stack.pop()
                # child link
                child_link = top_joint.get_child()
                # get all child joints of child_link
                # this are the "child joints" of top_joint as well
                child_joints = robot.get_joints_by_parent_name(child_link)
                # these joints will always be unvisited because tree so we add to stack
                for child_joint in child_joints:
                    stack.append(child_joint)
                # for preorder
                fproc_chain.append(top_joint)

            fproc_chain_by_jid[curr_chain_jid] = fproc_chain

        # print chains
        for jid in range(robot.get_num_joints()):
            fproc_chain = fproc_chain_by_jid[jid]
            # have +1 for link id
            fproc_chain_jids = [joint.get_id()+1 for joint in fproc_chain]
            #print(str(jid+1)+": "+str(fproc_chain_jids))

        return fproc_chain_by_jid

    def get_bproc_chains(self):
        robot = self.robot
        robot_joints = robot.get_joints_ordered_by_id()

        bproc_chain_by_jid = {}

        # DFS from every joint
        for curr_chain_joint in robot_joints:
            curr_chain_jid = curr_chain_joint.get_id()
            # fproc chain is all the joints visited from curr_chain_joint in preorder traversal
            bproc_chain = []

            stack = []
            stack.append(curr_chain_joint)
            while stack:
                top_joint = stack.pop()
                # child link
                child_link = top_joint.get_child()
                # get all child joints of child_link
                # this are the "child joints" of top_joint as well
                child_joints = robot.get_joints_by_parent_name(child_link)
                # these joints will always be unvisited because tree so we add to stack
                for child_joint in child_joints:
                    stack.append(child_joint)
                # we eventually reverse the list below
                bproc_chain.append(top_joint)

                # append world link if no child_joints
                if not child_joints:
                    world_joint = Joint(
                            name="world",
                            # world joint jid is larger than
                            # all other jids
                            jid=robot.get_num_joints(),
                            parent=child_link,
                            child="None"
                    )
                    bproc_chain.append(world_joint)
            # reversing for postorder traversal
            bproc_chain = bproc_chain[::-1]

            # now we climb up to root of tree
            has_parent = True
            parent_joint = curr_chain_joint
            while has_parent:
                parent_joint = self.get_parent_joint(parent_joint)
                if parent_joint:
                    bproc_chain.append(parent_joint)
                else:
                    has_parent = False
                    break

            bproc_chain_by_jid[curr_chain_jid] = bproc_chain

        # print chains
        for jid in range(robot.get_num_joints()):
            bproc_chain = bproc_chain_by_jid[jid]
            # have +1 for link id
            bproc_chain_jids = [joint.get_id()+1 for joint in bproc_chain]
            #print(str(jid+1)+": "+str(bproc_chain_jids))

        return bproc_chain_by_jid

    def longestPath(self, root_task, task_graph):
        if root_task.comes_before.is_empty():
            return []

        child_paths = []
        for child_task in root_task.comes_before.get_task_list():
            child_task_in_graph = task_graph.retrieve_task(child_task)
            child_longest_path = self.longestPath(child_task_in_graph, task_graph)
            child_paths.append(child_longest_path)

        # insert root node to longest path
        longest_path_ind = -1
        longest_path_len = -1
        for i in range(len(child_paths)):
            path = child_paths[i]
            if len(path) > longest_path_len:
                longest_path_len = len(path)
                longest_path_ind = i

        longest_path = child_paths[longest_path_ind]
        longest_path.append(root_task)
        return longest_path


    def set_task_graph_dist(self, task_graph):
        # keeping separate graph for updating comes_before values
        # because of python copy-ing nonsense
        task_graph_with_comes_before = copy.deepcopy(task_graph)

        for task in task_graph.get_task_list():
            for prereq in task.prereqs.get_task_list():
                prereq_in_graph = task_graph_with_comes_before.remove(prereq)
                prereq_in_graph.comes_before.add(task)
                task_graph_with_comes_before.add(prereq_in_graph)

        task_graph = task_graph_with_comes_before
        task_graph_with_dist = copy.deepcopy(task_graph)

        for task in task_graph.get_task_list():
            # DFS through task graph from each task and find longest
            # path to leaf
            # `comes_before` is set of directed edges going from this task to next tasks
            # task in `comes_before` depend on this task
            task_to_upd = task_graph_with_dist.remove(task)
            longest_path = self.longestPath(task, task_graph)
            task_to_upd.set_dist_from_sink(len(longest_path))
            task_graph_with_dist.add(task_to_upd)

        return task_graph_with_dist
        
    def fproc_chain_to_task_graph(self):
        robot = self.robot
        robot_joints = robot.get_joints_ordered_by_id()

        fproc_chain_by_jid = self.get_fproc_chains() 

        fproc_task_graph = SchedTaskList()

        for curr_ID_jid in range(robot.get_num_joints()):
            par_ID_joint = self.get_parent_joint(robot.get_joint_by_id(curr_ID_jid))

            par_ID_jid = -1
            if par_ID_joint:
                par_ID_jid = par_ID_joint.get_id()

            ID_task = Task(
                    resource="ID",
                    curr=curr_ID_jid+1,
                    par=par_ID_jid+1
            )
            if par_ID_joint:
                par_par_ID_jid = -1
                par_par_ID_joint = self.get_parent_joint(par_ID_joint)
                if par_par_ID_joint:
                    # if no par_par_ID_joint, it means par_ID_joint is root joint, so par_par_ID_jid+1 is just 0
                    par_par_ID_jid = par_par_ID_joint.get_id()
                ID_task.set_prereq(fproc_task_graph.find_ID_task(par_ID_jid+1, par_par_ID_jid+1))
            fproc_task_graph.add(ID_task)

        for derv_dID_jid in range(robot.get_num_joints()):
            fproc_chain = fproc_chain_by_jid[derv_dID_jid]
            for curr_dID_joint in fproc_chain:
                curr_dID_jid = curr_dID_joint.get_id()

                par_dID_joint = self.get_parent_joint(curr_dID_joint)

                par_dID_jid = -1
                if par_dID_joint:
                    par_dID_jid = par_dID_joint.get_id()

                dID_task = Task(
                        resource="dID",
                        # +1'd because we actually schedule 1-indexed links
                        # 0th link is usually the base link
                        curr=curr_dID_jid+1,
                        derv=derv_dID_jid+1,
                        par=par_dID_jid+1
                )

                dID_task.set_prereq(fproc_task_graph.find_ID_task(curr_dID_jid+1, par_dID_jid+1))
                if par_dID_joint and par_dID_jid in self.joint_chain_by_id(fproc_chain):
                    dID_task.set_prereq(fproc_task_graph.find_dID_task(par_dID_jid+1, derv_dID_jid+1))
                fproc_task_graph.add(dID_task)

        fproc_task_graph = self.set_task_graph_dist(fproc_task_graph)
        
        # print task graph
        #fproc_task_graph.print_task_graph()

        return fproc_task_graph

    def bproc_chain_to_task_graph(self):
        robot = self.robot

        bproc_chain_by_jid = self.get_bproc_chains() 

        bproc_task_graph = SchedTaskList()

        for curr_ID_jid in range(robot.get_num_joints()):
            par_ID_joint = self.get_parent_joint(robot.get_joint_by_id(curr_ID_jid))

            par_ID_jid = -1
            if par_ID_joint:
                par_ID_jid = par_ID_joint.get_id()

            ID_task = Task(
                    resource="ID",
                    curr=curr_ID_jid+1,
                    par=par_ID_jid+1
            )
            
            # prereqs for bproc is children
            curr_ID_joint = robot.get_joint_by_id(curr_ID_jid)
            children_joints = self.get_children_joints_by_parent_joint(curr_ID_joint)
            if children_joints:
                for child_joint in children_joints:
                    child_joint_jid = child_joint.get_id()
                    child_task = Task(
                            resource="ID",
                            # +1'd because we actually schedule 1-indexed links
                            # 0th link is usually the base link
                            curr=child_joint_jid+1,
                            par=curr_ID_jid+1
                    )
                    ID_task.set_prereq(child_task)
            else:
                # we add a world joint task as a prereq
                world_task = Task(
                        resource="ID",
                        curr=robot.get_num_joints()+1,
                        par=curr_ID_jid+1
                )
                ID_task.set_prereq(world_task)
                bproc_task_graph.add(world_task)

            bproc_task_graph.add(ID_task)

        for derv_dID_jid in range(robot.get_num_joints()):
            bproc_chain = bproc_chain_by_jid[derv_dID_jid]
            for curr_dID_joint in bproc_chain:
                curr_dID_jid = curr_dID_joint.get_id()

                par_dID_joint = self.get_parent_joint(curr_dID_joint)

                par_dID_jid = -1
                if par_dID_joint:
                    par_dID_jid = par_dID_joint.get_id()

                dID_task = Task(
                        resource="dID",
                        # +1'd because we actually schedule 1-indexed links
                        # 0th link is usually the base link
                        curr=curr_dID_jid+1,
                        derv=derv_dID_jid+1,
                        par=par_dID_jid+1
                )

                children_joints = self.get_children_joints_by_parent_joint(curr_dID_joint)
                if children_joints:
                    for child_joint in children_joints:
                        child_joint_jid = child_joint.get_id()
                        dID_task.set_prereq(bproc_task_graph.find_ID_task(child_joint_jid+1, curr_dID_jid+1))
                        if child_joint_jid in self.joint_chain_by_id(bproc_chain):
                            dID_task.set_prereq(bproc_task_graph.find_dID_task(child_joint_jid+1, derv_dID_jid+1))
                else:
                    if not curr_dID_joint.get_name() == "world":
                        # leaf nodes which officially have no children in robot
                        # but are connected to a "virtual" world joint
                        dID_task.set_prereq(bproc_task_graph.find_ID_task(robot.get_num_joints()+1, curr_dID_jid+1))
                        dID_task.set_prereq(bproc_task_graph.find_dID_task(robot.get_num_joints()+1, derv_dID_jid+1))


                bproc_task_graph.add(dID_task)

        bproc_task_graph = self.set_task_graph_dist(bproc_task_graph)
        
        # print task graph
        #bproc_task_graph.print_task_graph()

        return bproc_task_graph

    def is_prereqs_scheduled(self, task, scheduled_task_graph):
        prereqs = task.prereqs.get_task_list()
        for prereq in prereqs:
            if not scheduled_task_graph.retrieve_task(prereq):
                return False
        return True

    def is_descendent_before_join_ID(self, descendent, parent, task_graph):
        # DFS-ing through task graph from parent to check if descendent found
        # before_join means if some node is a join operation (with > 1 parent)
        # we do not DFS further
        # The join operation really just happens for ID backward pass
        descendent_found = False
        stack = []
        stack.append(parent)
        while stack:
            t = stack.pop()
            if t.curr == descendent.curr and t.par == descendent.par:
                descendent_found = True
                break
            for child_task in t.comes_before.get_task_list():
                if child_task.resource == "ID":
                    # check if child has two ID parents (ie. is join)
                    n_child_ID_parents = 0
                    for child_parent_task in child_task.prereqs.get_task_list():
                        if child_parent_task.resource == "ID":
                            n_child_ID_parents += 1
                            if n_child_ID_parents == 2:
                                break
                    if n_child_ID_parents <= 1:
                        stack.append(task_graph.retrieve_task(child_task))
        return descendent_found

    def schedule_onto_resources(self, task_graph, num_ID_PEs, num_dID_PEs):
        assert num_ID_PEs > 0, "num_PEs must be > 0"
        assert num_dID_PEs > 0, "num_PEs must be > 0"

        ID_scheds_by_PE_id = {}
        dID_scheds_by_PE_id = {}
        for PE_id in range(num_ID_PEs):
            ID_scheds_by_PE_id[PE_id] = []
        for PE_id in range(num_dID_PEs):
            dID_scheds_by_PE_id[PE_id] = []
        
        to_sched_list = SchedTaskList()
        for t in task_graph.get_task_list_ordered_by_dist_from_sink():
            to_sched_list.add(t)
        scheduled_list = SchedTaskList()
        
        step = 0

        while not to_sched_list.is_empty():
            # tracking tasks scheduled in this step so that we don't
            # count them as prereqs met for task in same step
            tasks_for_step = SchedTaskList()

            # for each PE, we try to find a suitable task to schedule onto it
            for PE_id in range(num_ID_PEs):
                found_task_for_step = False
                task_for_step = None
                for candidate_task in to_sched_list.get_task_list():
                    if candidate_task.resource == "ID":
                        if self.is_prereqs_scheduled(candidate_task, scheduled_list):
                            ID_sched = ID_scheds_by_PE_id[PE_id]
                            # must continue chain whenever possible
                            prev_task = None
                            if ID_sched:
                                prev_task = ID_sched[-1]
                            if prev_task is None:
                                task_for_step = candidate_task
                                found_task_for_step = True
                                break
                            else:
                                # check if candidate task is part of ongoing chain
                                if self.is_descendent_before_join_ID(candidate_task, prev_task, task_graph):
                                    task_for_step = candidate_task
                                    found_task_for_step = True
                                    break
                                else:
                                    # check if current chain is complete and we can start a new chain
                                    chain_complete = True
                                    for remaining_chain_task in to_sched_list.get_task_list():
                                        if remaining_chain_task.resource == "ID" and self.is_descendent_before_join_ID(remaining_chain_task, prev_task, task_graph):
                                            # there are tasks remaining to be scheduled on this PE
                                            # for the ongoing chain, we can't schedule candidate
                                            # task onto this PE
                                            chain_complete = False
                                            break
                                    if chain_complete:
                                        task_for_step = candidate_task
                                        found_task_for_step = True
                                        break

                if not found_task_for_step:
                    # nop
                    nop_task = Task(
                            resource="ID",
                            curr=0,
                            par=0
                    )
                    ID_scheds_by_PE_id[PE_id].append(nop_task)
                else:
                    to_sched_list.remove(task_for_step)
                    tasks_for_step.add(task_for_step)
                    ID_scheds_by_PE_id[PE_id].append(task_for_step)

            for PE_id in range(num_dID_PEs):
                found_task_for_step = False
                task_for_step = None
                for candidate_task in to_sched_list.get_task_list():
                    if candidate_task.resource == "dID":
                        if self.is_prereqs_scheduled(candidate_task, scheduled_list):
                            dID_sched = dID_scheds_by_PE_id[PE_id]
                            # check if any chain in progress and if we would be breaking chain
                            # must continue chain whenever possible for dID
                            prev_not_NOP_task = None
                            for t in dID_sched:
                                if t.curr != 0:
                                    # not NOP
                                    prev_not_NOP_task = t
                            if prev_not_NOP_task is None:
                                # only NOPs on this PE so far
                                task_for_step = candidate_task
                                found_task_for_step = True
                                break
                            else:
                                PE_id_derv = prev_not_NOP_task.derv
                                if candidate_task.derv == PE_id_derv:
                                    task_for_step = candidate_task
                                    found_task_for_step = True
                                    break
                                else:
                                    # if not, check if chain is complete and we start new one
                                    chain_complete = True
                                    for remaining_chain_task in to_sched_list.get_task_list():
                                        if remaining_chain_task.derv == PE_id_derv:
                                            # there are tasks remaining to be scheduled on this PE
                                            # for the ongoing chain, we can't schedule candidate
                                            # task onto this PE
                                            chain_complete = False
                                            break
                                    if chain_complete:
                                        # need to make sure chain for candidate_task isn't running anywhere else
                                        ongoing_chain_elsewhere = False
                                        for other_PE_id in range(num_dID_PEs):
                                            if other_PE_id != PE_id:
                                                other_dID_sched = dID_scheds_by_PE_id[other_PE_id]
                                                for other_t in other_dID_sched:
                                                    if other_t.derv == candidate_task.derv:
                                                        ongoing_chain_elsewhere = True
                                                        break
                                        if not ongoing_chain_elsewhere:
                                            task_for_step = candidate_task
                                            found_task_for_step = True
                                            break
                if not found_task_for_step:
                    # nop
                    nop_task = Task(
                            resource="dID",
                            curr=0,
                            par=0,
                            derv=0
                    )
                    dID_scheds_by_PE_id[PE_id].append(nop_task)
                else:
                    to_sched_list.remove(task_for_step)
                    tasks_for_step.add(task_for_step)
                    dID_scheds_by_PE_id[PE_id].append(task_for_step)

            scheduled_list.extend(tasks_for_step)

            step += 1

        # printing schedules
        currs_for_derv = {}
        for task in task_graph.get_task_list():
            if task.derv not in currs_for_derv:
                currs_for_derv[task.derv] = []
            currs_for_derv[task.derv].append(task.curr)

       # for derv in currs_for_derv.keys():
       #     print(str(derv)+": "+str(currs_for_derv[derv]))

       # frmt = "{:>3}"*step

       # for PE_id in range(num_ID_PEs):
       #     ID_sched = ID_scheds_by_PE_id[PE_id]
       #     ID_sched_curr = [t.curr for t in ID_sched]
       #     print("ID:       "+frmt.format(*ID_sched_curr))
       # for PE_id in range(num_dID_PEs):
       #     dID_sched = dID_scheds_by_PE_id[PE_id]
       #     dID_sched_curr = [t.curr for t in dID_sched]
       #     dID_sched_derv = [t.derv for t in dID_sched]
       #     dID_sched_par = [t.par for t in dID_sched]
       #     print("PE"+str(PE_id+1)+" curr: "+frmt.format(*dID_sched_curr))
       #     print("PE"+str(PE_id+1)+" derv: "+frmt.format(*dID_sched_derv))
       #     print("PE"+str(PE_id+1)+" par:  "+frmt.format(*dID_sched_par))

        return ID_scheds_by_PE_id, dID_scheds_by_PE_id

    def sanitize_NOPs(self, dID_scheds_by_PE_id):
        # we aren't allowed to have derv == 0, so we mark dervs for NOPs
        # as next non-zero derv in PE sched
        for PE_id in dID_scheds_by_PE_id.keys():
            dID_sched = dID_scheds_by_PE_id[PE_id]
            dID_sched_copy = []

            last_non_NOP_derv = -1
            for dID_step,dID_task in enumerate(dID_sched):
                if dID_task.curr != 0:
                    last_non_NOP_derv = dID_task.derv
                else:
                    # NOP
                    if last_non_NOP_derv == -1:
                        next_non_NOP_derv = -1
                        for next_non_NOP_step in range(dID_step+1, len(dID_sched)):
                            next_non_NOP_task = dID_sched[next_non_NOP_step]
                            if next_non_NOP_task.curr != 0:
                                next_non_NOP_derv = next_non_NOP_task.derv
                                break
                        if next_non_NOP_derv == -1:
                            # entire PE sched was just NOPs
                            # just mark anything as derv
                            next_non_NOP_derv = 1
                        dID_task.derv = next_non_NOP_derv
                    else:
                        dID_task.derv = last_non_NOP_derv
                dID_sched_copy.append(copy.deepcopy(dID_task))
            dID_scheds_by_PE_id[PE_id] = dID_sched_copy

        return dID_scheds_by_PE_id

    def get_matmul_schedule_tables(self, num_PEs=-1):
        if num_PEs == -1:
            num_PEs = self.num_bproc_PEs

        robot = self.robot
        matmul_block_size = self.block_size

        num_joints = robot.get_num_joints()

        # cooking up strawman minv
        q = np.random.rand((num_joints))
        rbd_reference = rbdReference(robot)
        Minv = rbd_reference.minv(q)

        print(Minv)
        
        # top left corner
        non_zero_block_start_r = []
        non_zero_block_start_c = []
        non_zero_block_size = []

        row_idx = 0
        col_idx = 0
        non_zero_block_ongoing = False
        curr_block_size = 0
        while col_idx < num_joints:
            if not np.allclose(Minv[row_idx, col_idx], 0):
                if not non_zero_block_ongoing:
                    non_zero_block_start_r.append(row_idx)
                    non_zero_block_start_c.append(col_idx)
                    non_zero_block_ongoing = True
                curr_block_size += 1
                col_idx += 1
            else:
                if non_zero_block_ongoing:
                    non_zero_block_size.append(curr_block_size)
                    non_zero_block_ongoing = False
                    row_idx += curr_block_size
                    curr_block_size = 0

        # if last block found hasn't completed
        if non_zero_block_ongoing:
            non_zero_block_size.append(curr_block_size)

        #print(non_zero_block_start_r)
        #print(non_zero_block_start_c)
        #print(non_zero_block_size)

        # check if any zeros within each matmul block boundary
        nonzero_matmul_block_r = []
        nonzero_matmul_block_c = []

        last_ind = int(ceil(num_joints/matmul_block_size)) * matmul_block_size
        num_matmul_blocks = last_ind // matmul_block_size

        for matmul_block_r in range(0, last_ind, matmul_block_size):
            for matmul_block_c in range(0, last_ind, matmul_block_size):
                matmul_top_right = (matmul_block_r, matmul_block_c+matmul_block_size)
                matmul_bottom_left = (matmul_block_r+matmul_block_size, matmul_block_c)
                for non_zero_block_idx in range(len(non_zero_block_size)):
                    non_zero_block_start_r_idx = non_zero_block_start_r[non_zero_block_idx]
                    non_zero_block_start_c_idx = non_zero_block_start_c[non_zero_block_idx]
                    non_zero_block_size_idx = non_zero_block_size[non_zero_block_idx]
                    # rect-rect intersection test
                    non_zero_block_top_right = (non_zero_block_start_r_idx, non_zero_block_start_c_idx+non_zero_block_size_idx)
                    non_zero_block_bottom_left = (non_zero_block_start_r_idx+non_zero_block_size_idx, non_zero_block_start_c_idx)
                    is_overlap = not (matmul_top_right[1] <= non_zero_block_bottom_left[1] or \
                            matmul_bottom_left[1] >= non_zero_block_top_right[1] or \
                            matmul_top_right[0] >= non_zero_block_bottom_left[0] or \
                            matmul_bottom_left[0] <= non_zero_block_top_right[0]
                    )
                    if is_overlap:
                        nonzero_matmul_block_r.append(matmul_block_r)
                        nonzero_matmul_block_c.append(matmul_block_c)
                        break

        #print(nonzero_matmul_block_r)
        #print(nonzero_matmul_block_c)
        num_nonzero_blocks = len(nonzero_matmul_block_r)

        blocked_mat_pattern = np.zeros((num_matmul_blocks, num_matmul_blocks))
        for i in range(num_nonzero_blocks):
            nonzero_block_r = nonzero_matmul_block_r[i] // matmul_block_size
            nonzero_block_c = nonzero_matmul_block_c[i] // matmul_block_size
            blocked_mat_pattern[nonzero_block_r, nonzero_block_c] = 1

        n_tiles = int(np.sum(blocked_mat_pattern))
        n_matmuls = int(np.sum(np.matmul(blocked_mat_pattern, blocked_mat_pattern)))
        n_steps = int(ceil((n_matmuls * matmul_block_size) / num_PEs))

        # each 1 entry in blocked_mat_pattern corresponds to contiguous block of non-zeros
        # in minv
        # we now generate schedule for tile x tile multiply and accumulate
        # doing right-multiplication from here:
        # https://eli.thegreenplace.net/2015/visualizing-matrix-multiplication-as-a-linear-combination/
        lh_tile_sched_r = []
        lh_tile_sched_c = []
        rh_tile_sched_r = []
        rh_tile_sched_c = []
        output_tile_acc_sched_r = []
        output_tile_acc_sched_c = []

        for rh_tile_col_idx in range(num_matmul_blocks):
            for rh_tile_row_idx in range(num_matmul_blocks):
                if blocked_mat_pattern[rh_tile_row_idx, rh_tile_col_idx] == 1:
                    # found a nonzero block in rh
                    lh_tile_col_idx = rh_tile_row_idx
                    for lh_tile_row_idx in range(num_matmul_blocks):
                        if blocked_mat_pattern[lh_tile_row_idx, lh_tile_col_idx] == 1:
                            lh_tile_sched_r.append(lh_tile_row_idx)
                            lh_tile_sched_c.append(lh_tile_col_idx)
                            rh_tile_sched_r.append(rh_tile_row_idx)
                            rh_tile_sched_c.append(rh_tile_col_idx)
                            output_tile_acc_sched_r.append(lh_tile_row_idx)
                            output_tile_acc_sched_c.append(rh_tile_col_idx)

        assert n_matmuls == len(lh_tile_sched_r), "messed up tile sched"

        # blocked matmul in hardware is implemented as `num_PEs` matrix-col vector multiplies 
        # so each PE accepts an entire lh block and a single rh col from rh block
        # for each PE per schedule step, we schedule an lh block and an rh col from some rh block

        # lh block top left corner for each block processed by each PE
        lh_tile_row_start_sched = []
        lh_tile_col_start_sched = []
        # rh block top left corner
        rh_tile_row_start_sched = []
        rh_tile_col_start_sched = []
        # rh tile col sched
        rh_tile_col_sched = []
        # output block top left corner
        output_tile_row_start_sched = []
        output_tile_col_start_sched = []

        step = 0
        curr_PE_for_step = 0

        lh_tile_row_start_step = []
        lh_tile_col_start_step = []
        rh_tile_col_step = []
        rh_tile_row_start_step = []
        rh_tile_col_start_step = []
        output_tile_row_start_step = []
        output_tile_col_start_step = []

        for tile_sched_idx in range(n_matmuls):
            for rh_col_idx in range(matmul_block_size):
                lh_tile_row_start_step.append(lh_tile_sched_r[tile_sched_idx]*matmul_block_size)
                lh_tile_col_start_step.append(lh_tile_sched_c[tile_sched_idx]*matmul_block_size)

                rh_tile_col_step.append(rh_col_idx)
                rh_tile_row_start_step.append(rh_tile_sched_r[tile_sched_idx]*matmul_block_size)
                rh_tile_col_start_step.append(rh_tile_sched_c[tile_sched_idx]*matmul_block_size)

                output_tile_row_start_step.append(output_tile_acc_sched_r[tile_sched_idx]*matmul_block_size)
                output_tile_col_start_step.append(output_tile_acc_sched_c[tile_sched_idx]*matmul_block_size)

                curr_PE_for_step += 1
                if curr_PE_for_step == num_PEs or (tile_sched_idx == n_matmuls-1 and rh_col_idx == matmul_block_size-1):
                    if tile_sched_idx == n_matmuls-1 and rh_col_idx == matmul_block_size-1:
                        # last tile and last col
                        # pad leftover PEs with NOPs
                        padding = num_PEs - curr_PE_for_step
                        lh_tile_row_start_step.extend([matmul_block_size]*padding)
                        lh_tile_col_start_step.extend([matmul_block_size]*padding)

                        # `block_size` is NOP for rh_tile_col
                        rh_tile_col_step.extend([matmul_block_size]*padding)
                        rh_tile_row_start_step.extend([matmul_block_size]*padding)
                        rh_tile_col_start_step.extend([matmul_block_size]*padding)

                        output_tile_row_start_step.extend([matmul_block_size]*padding)
                        output_tile_col_start_step.extend([matmul_block_size]*padding)

                    lh_tile_row_start_sched.append(lh_tile_row_start_step)
                    lh_tile_col_start_sched.append(lh_tile_col_start_step)

                    rh_tile_col_sched.append(rh_tile_col_step)
                    rh_tile_row_start_sched.append(rh_tile_row_start_step)
                    rh_tile_col_start_sched.append(rh_tile_col_start_step)

                    output_tile_row_start_sched.append(output_tile_row_start_step)
                    output_tile_col_start_sched.append(output_tile_col_start_step)

                    curr_PE_for_step = 0
                    step += 1

                    lh_tile_row_start_step = []
                    lh_tile_col_start_step = []

                    rh_tile_col_step = []
                    rh_tile_row_start_step = []
                    rh_tile_col_start_step = []

                    output_tile_row_start_step = []
                    output_tile_col_start_step = []

        assert n_steps == step, "messed up step PE sched"

        matmul_sched_table_dict = {}

        matmul_sched_table_dict["n_tiles"] = n_tiles
        matmul_sched_table_dict["len_block_minv_sched_per_matrix"] = n_steps

        lh_tile_sched = []
        rh_tile_sched = []
        output_tile_incr_sched = []
        for step in range(n_steps):
            lh_tile_sched_step = []
            rh_tile_sched_step = []
            output_tile_incr_sched_step = []
            for PE_id in range(num_PEs):
                lh_tile_id = (lh_tile_row_start_sched[step][PE_id] // matmul_block_size) * num_matmul_blocks + (lh_tile_col_start_sched[step][PE_id] // matmul_block_size)
                rh_tile_id = (rh_tile_row_start_sched[step][PE_id] // matmul_block_size) * num_matmul_blocks + (rh_tile_col_start_sched[step][PE_id] // matmul_block_size)
                output_tile_incr_id = (output_tile_row_start_sched[step][PE_id] // matmul_block_size) * num_matmul_blocks + (output_tile_col_start_sched[step][PE_id] // matmul_block_size)

                lh_tile_sched_step.append(lh_tile_id)
                rh_tile_sched_step.append(rh_tile_id)
                output_tile_incr_sched_step.append(output_tile_incr_id)
            lh_tile_sched.append(lh_tile_sched_step)
            rh_tile_sched.append(rh_tile_sched_step)
            output_tile_incr_sched.append(output_tile_incr_sched_step)
        matmul_sched_table_dict["lh_tile_sched"] = lh_tile_sched
        matmul_sched_table_dict["rh_tile_sched"] = rh_tile_sched
        matmul_sched_table_dict["output_tile_incr_sched"] = output_tile_incr_sched

        matmul_sched_table_dict["lh_tile_row_start_sched"] = lh_tile_row_start_sched
        matmul_sched_table_dict["lh_tile_col_start_sched"] = lh_tile_col_start_sched

        matmul_sched_table_dict["rh_tile_col_sched"] = rh_tile_col_sched
        matmul_sched_table_dict["rh_tile_row_start_sched"] = rh_tile_row_start_sched
        matmul_sched_table_dict["rh_tile_col_start_sched"] = rh_tile_col_start_sched
        matmul_sched_table_dict["output_tile_row_start_sched"] = output_tile_row_start_sched
        matmul_sched_table_dict["output_tile_col_start_sched"] = output_tile_col_start_sched

        return matmul_sched_table_dict

    def get_fproc_schedule_tables(self):
        fproc_task_graph = self.fproc_chain_to_task_graph()
        ID_scheds_by_PE_id, dID_scheds_by_PE_id = self.schedule_onto_resources(fproc_task_graph, 1, self.num_fproc_PEs)

        dID_scheds_by_PE_id = self.sanitize_NOPs(dID_scheds_by_PE_id)

        len_fproc_sched = len(ID_scheds_by_PE_id[0])

        assert len(ID_scheds_by_PE_id.keys()) == 1, "ID num_PEs must be 1"
        rnea_fproc_curr_sched = []
        rnea_fproc_par_sched = []
        for step in range(len_fproc_sched):
            rnea_fproc_curr_sched.append(ID_scheds_by_PE_id[0][step].curr)
            rnea_fproc_par_sched.append(ID_scheds_by_PE_id[0][step].par)

        fproc_curr_sched = []
        fproc_derv_sched = []
        fproc_par_sched = []
        for step in range(len_fproc_sched):
            curr_sched_step = []
            derv_sched_step = []
            par_sched_step = []
            for PE_id in range(self.num_fproc_PEs):
                curr_sched_step.append(dID_scheds_by_PE_id[PE_id][step].curr)
                derv_sched_step.append(dID_scheds_by_PE_id[PE_id][step].derv)
                par_sched_step.append(dID_scheds_by_PE_id[PE_id][step].par)
            fproc_curr_sched.append(curr_sched_step)
            fproc_derv_sched.append(derv_sched_step)
            fproc_par_sched.append(par_sched_step)

        sched_table_dict = {}
        sched_table_dict["type"] = "fproc"
        sched_table_dict["len_fproc_sched"] = len_fproc_sched
        sched_table_dict["rnea_fproc_curr_sched"] = rnea_fproc_curr_sched
        sched_table_dict["rnea_fproc_par_sched"] = rnea_fproc_par_sched
        sched_table_dict["fproc_curr_sched"] = fproc_curr_sched
        sched_table_dict["fproc_derv_sched"] = fproc_derv_sched
        sched_table_dict["fproc_par_sched"] = fproc_par_sched

        return sched_table_dict

    def get_bproc_schedule_tables(self):
        bproc_task_graph = self.bproc_chain_to_task_graph()
        ID_scheds_by_PE_id, dID_scheds_by_PE_id = self.schedule_onto_resources(bproc_task_graph, 1, self.num_bproc_PEs)

        dID_scheds_by_PE_id = self.sanitize_NOPs(dID_scheds_by_PE_id)

        len_bproc_sched = len(ID_scheds_by_PE_id[0])

        assert len(ID_scheds_by_PE_id.keys()) == 1, "ID num_PEs must be 1"
        rnea_bproc_curr_sched = []
        rnea_bproc_par_sched = []
        for step in range(len_bproc_sched):
            rnea_bproc_curr_sched.append(ID_scheds_by_PE_id[0][step].curr)
            rnea_bproc_par_sched.append(ID_scheds_by_PE_id[0][step].par)

        bproc_curr_sched = []
        bproc_derv_sched = []
        bproc_par_sched = []
        for step in range(len_bproc_sched):
            curr_sched_step = []
            derv_sched_step = []
            par_sched_step = []
            for PE_id in range(self.num_bproc_PEs):
                curr_sched_step.append(dID_scheds_by_PE_id[PE_id][step].curr)
                derv_sched_step.append(dID_scheds_by_PE_id[PE_id][step].derv)
                par_sched_step.append(dID_scheds_by_PE_id[PE_id][step].par)
            bproc_curr_sched.append(curr_sched_step)
            bproc_derv_sched.append(derv_sched_step)
            bproc_par_sched.append(par_sched_step)

        sched_table_dict = {}
        sched_table_dict["type"] = "bproc"
        sched_table_dict["len_bproc_sched"] = len_bproc_sched
        sched_table_dict["rnea_bproc_curr_sched"] = rnea_bproc_curr_sched
        sched_table_dict["rnea_bproc_par_sched"] = rnea_bproc_par_sched
        sched_table_dict["bproc_curr_sched"] = bproc_curr_sched
        sched_table_dict["bproc_derv_sched"] = bproc_derv_sched
        sched_table_dict["bproc_par_sched"] = bproc_par_sched

        return sched_table_dict

    def print_fmted_schedules(self, sched_table_dict):
        len_sched = -1
        rnea_curr_sched = None
        rnea_par_sched = None
        curr_sched = None
        derv_sched = None
        par_sched = None
        if sched_table_dict["type"] == "fproc":
            len_sched = sched_table_dict["len_fproc_sched"]
            rnea_curr_sched = sched_table_dict["rnea_fproc_curr_sched"]
            rnea_par_sched = sched_table_dict["rnea_fproc_par_sched"]
            curr_sched = sched_table_dict["fproc_curr_sched"]
            derv_sched = sched_table_dict["fproc_derv_sched"]
            par_sched = sched_table_dict["fproc_par_sched"]
        if sched_table_dict["type"] == "bproc":
            len_sched = sched_table_dict["len_bproc_sched"]
            rnea_curr_sched = sched_table_dict["rnea_bproc_curr_sched"]
            rnea_par_sched = sched_table_dict["rnea_bproc_par_sched"]
            curr_sched = sched_table_dict["bproc_curr_sched"]
            derv_sched = sched_table_dict["bproc_derv_sched"]
            par_sched = sched_table_dict["bproc_par_sched"]
        
        num_PEs = len(curr_sched[0])

        print("--------------")

        header_str = "| ID ||"
        for PE_id in range(num_PEs):
            header_str += " c{PE_id} | d{PE_id} ||".format(PE_id=PE_id+1)
        print(header_str)

        PE_frmt = " {:>2} | {:>2} ||"

        for step in range(len_sched):
            row_str = ""
            row_str += "| {:>2} ||".format(rnea_curr_sched[step]) 
            for PE_id in range(num_PEs):
                row_str += PE_frmt.format(curr_sched[step][PE_id], derv_sched[step][PE_id])
            print(row_str)
        print("--------------")

    def print_matmul_schedules(self, matmul_sched_table_dict):
        lh_tile_sched = matmul_sched_table_dict["lh_tile_sched"]
        len_sched = len(lh_tile_sched)
        num_PEs = len(lh_tile_sched[0])

        for sched_name in ["lh_tile_sched", "rh_tile_sched", "output_tile_incr_sched", "rh_tile_col_sched"]:
            sched = matmul_sched_table_dict[sched_name]

            print("----------")
            print(sched_name+":")
            header_str = "|"
            for PE_id in range(num_PEs):
                header_str += " PE{PE_id} |".format(PE_id=PE_id+1)
            print(header_str)

            PE_frmt = " {:>3} |"

            for step in range(len_sched):
                row_str = "|"
                for PE_id in range(num_PEs):
                    row_str += PE_frmt.format(sched[step][PE_id])
                print(row_str)
            print("---------")

    def write_schedule_table_csv(self, sched_table_dict, csvdir_path):
        len_sched = -1
        rnea_curr_sched = None
        rnea_par_sched = None
        curr_sched = None
        derv_sched = None
        par_sched = None
        if sched_table_dict["type"] == "fproc":
            len_sched = sched_table_dict["len_fproc_sched"]
            rnea_curr_sched = sched_table_dict["rnea_fproc_curr_sched"]
            rnea_par_sched = sched_table_dict["rnea_fproc_par_sched"]
            curr_sched = sched_table_dict["fproc_curr_sched"]
            derv_sched = sched_table_dict["fproc_derv_sched"]
            par_sched = sched_table_dict["fproc_par_sched"]
        if sched_table_dict["type"] == "bproc":
            len_sched = sched_table_dict["len_bproc_sched"]
            rnea_curr_sched = sched_table_dict["rnea_bproc_curr_sched"]
            rnea_par_sched = sched_table_dict["rnea_bproc_par_sched"]
            curr_sched = sched_table_dict["bproc_curr_sched"]
            derv_sched = sched_table_dict["bproc_derv_sched"]
            par_sched = sched_table_dict["bproc_par_sched"]
        
        sched_type = sched_table_dict["type"]
        num_PEs = len(curr_sched[0])

        os.makedirs(csvdir_path, exist_ok=True)
        csv_path = os.path.join(csvdir_path, sched_type+".csv")

        with open(csv_path, "w", newline='') as csvfile:
            csvwriter = csv.writer(csvfile, delimiter=",", quotechar="|", quoting=csv.QUOTE_MINIMAL)

            scheds_to_write = ["curr", "par", "derv"]

            for sched_name in scheds_to_write:
                csvwriter.writerow([sched_name])
                header_str_list = ["ID"]
                for PE_id in range(num_PEs):
                    header_str_list.append("PE"+str(PE_id+1))
                csvwriter.writerow(header_str_list)

                for step in range(len_sched):
                    sched_row = []
                    if sched_name == "curr":
                        sched_row.append(rnea_curr_sched[step])
                    elif sched_name == "par":
                        sched_row.append(rnea_par_sched[step])
                    elif sched_name == "derv":
                        sched_row.append(-1)

                    for PE_id in range(num_PEs):
                        if sched_name == "curr":
                            sched_row.append(curr_sched[step][PE_id])
                        elif sched_name == "par":
                            sched_row.append(par_sched[step][PE_id])
                        elif sched_name == "derv":
                            sched_row.append(derv_sched[step][PE_id])

                    csvwriter.writerow(sched_row)

    def write_matmul_schedule_table_csv(self, matmul_sched_table_dict, csvdir_path):
        lh_tile_sched = matmul_sched_table_dict["lh_tile_sched"]
        len_sched = len(lh_tile_sched)
        num_PEs = len(lh_tile_sched[0])

        os.makedirs(csvdir_path, exist_ok=True)
        csv_path = os.path.join(csvdir_path, "matmul.csv")

        with open(csv_path, "w", newline='') as csvfile:
            csvwriter = csv.writer(csvfile, delimiter=",", quotechar="|", quoting=csv.QUOTE_MINIMAL)

            for sched_name in ["lh_tile_sched", "rh_tile_sched", "output_tile_incr_sched", "rh_tile_col_sched"]:
                csvwriter.writerow([sched_name])

                header_str_list = []
                for PE_id in range(num_PEs):
                    header_str_list.append("PE"+str(PE_id+1))
                csvwriter.writerow(header_str_list)

                sched = matmul_sched_table_dict[sched_name]

                for step in range(len_sched):
                    sched_row = []
                    for PE_id in range(num_PEs):
                        sched_row.append(sched[step][PE_id])
                    csvwriter.writerow(sched_row)
