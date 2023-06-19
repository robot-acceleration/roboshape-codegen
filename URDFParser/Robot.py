from .Link import Link
from .Joint import Joint

class Robot:
    # initialization
    def __init__(self):
        self.links = []
        self.joints = []
        # Note: the following are directly accessed by URDFParser and
        # constructed for convenience in getting parent and subtree IDs
        # be careful when changing them. Everything else is accessed
        # by setters and getters.
        self.parent_lid_dict = {}
        self.subtree_lid_lists = {}

    #################
    #    Setters    #
    #################

    def add_joint(self, joint):
        self.joints.append(joint)

    def add_link(self, link):
        self.links.append(link)

    def remove_joint(self, joint):
        self.joints.remove(joint)

    def remove_link(self, link):
        self.links.remove(link)

    #########################
    #    Generic Getters    #
    #########################

    def get_num_pos(self):
        return len(self.joints)

    def get_num_vel(self):
        return len(self.joints)

    def get_parent_id(self, lid):
        return self.parent_lid_dict.get(lid)

    def get_parent_lid_array(self):
        return [tpl[1] for tpl in sorted(self.parent_lid_dict.items(), key=lambda tpl: tpl[0])]

    def get_subtree_by_id(self, lid):
        return self.subtree_lid_lists.get(lid)

    def get_max_bfs_level(self):
        return sorted(self.joints, key=lambda joint: joint.bfs_level, reverse = True)[0].bfs_level

    def get_ids_by_bfs_level(self, level):
        return [joint.jid for joint in self.get_joints_by_bfs_level(level)]


    ###############
    #    Joint    #
    ###############

    def get_num_joints(self):
        return len(self.joints)

    def get_joint_by_id(self, jid):
        for joint in self.joints:
            if joint.jid == jid:
                return joint
        return None

    def get_joint_by_name(self, name):
        for joint in self.joints:
            if joint.name == name:
                return joint
        return None

    def get_joints_by_bfs_level(self, level):
        return list(filter(lambda fjoint: fjoint.bfs_level == level, self.joints))

    def get_joints_ordered_by_id(self, reverse = False):
        return sorted(self.joints, key=lambda item: item.jid, reverse = reverse)

    def get_joints_ordered_by_name(self, reverse = False):
        return sorted(self.joints, key=lambda item: item.name, reverse = reverse)

    def get_joints_dict_by_id(self):
        return {joint.jid:joint for joint in self.joints}

    def get_joints_dict_by_name(self):
        return {joint.name:joint for joint in self.joints}

    def get_joints_by_parent_name(self, parent_name):
        return list(filter(lambda fjoint: fjoint.parent == parent_name, self.joints))

    def get_joints_by_child_name(self, child_name):
        return list(filter(lambda fjoint: fjoint.child == child_name, self.joints))

    def get_joint_by_parent_child_name(self, parent_name, child_name):
        return next(filter(lambda fjoint: fjoint.parent == parent_name and fjoint.child == child_name, self.joints))

    ##############
    #    Link    #
    ##############

    def get_num_links(self):
        return len(self.links)

    def get_num_links_effective(self):
        # subtracting base link from total # of links
        return self.get_num_links() - 1

    def get_link_by_id(self, lid):
        for link in self.links:
            if link.lid == lid:
                return link
        return None

    def get_link_by_name(self, name):
        for link in self.links:
            if link.name == name:
                return link
        return None

    def get_links_by_bfs_level(self, level):
        return list(filter(lambda flink: flink.bfs_level == level, self.link))

    def get_links_ordered_by_id(self, reverse = False):
        return sorted(self.links, key=lambda item: item.lid, reverse = reverse)

    def get_links_ordered_by_name(self, reverse = False):
        return sorted(self.links, key=lambda item: item.name, reverse = reverse)

    def get_links_dict_by_id(self):
        return {link.lid:link for link in self.links}

    def get_links_dict_by_name(self):
        return {link.name:link for link in self.links}

    ##############
    #    XMAT    #
    ##############

    def get_Xmat_by_id(self, jid):
        return self.get_joint_by_id(jid).get_transformation_matrix()

    def get_Xmat_by_name(self, name):
        return self.get_joint_by_name(name).get_transformation_matrix()

    def get_Xmats_by_bfs_level(self, level):
        return [joint.get_transformation_matrix() for joint in self.get_joints_by_bfs_level()]

    def get_Xmats_ordered_by_id(self, reverse = False):
        return [joint.get_transformation_matrix() for joint in self.get_joints_ordered_by_id(reverse)]

    def get_Xmats_ordered_by_name(self, reverse = False):
        return [joint.get_transformation_matrix() for joint in self.get_joints_ordered_by_name(reverse)]

    def get_Xmats_dict_by_id(self):
        return {joint.jid:joint.get_transformation_matrix() for joint in self.joints}

    def get_Xmats_dict_by_name(self):
        return {joint.name:joint.get_transformation_matrix() for joint in self.joints}

    ###################
    #    XMAT_Func    #
    ###################

    def get_Xmat_Func_by_id(self, jid):
        return self.get_joint_by_id(jid).get_transformation_matrix_function()

    def get_Xmat_Func_by_name(self, name):
        return self.get_joint_by_name(name).get_transformation_matrix_function()

    def get_Xmat_Funcs_by_bfs_level(self, level):
        return [joint.get_transformation_matrix_function() for joint in self.get_joints_by_bfs_level()]

    def get_Xmat_Funcs_ordered_by_id(self, reverse = False):
        return [joint.get_transformation_matrix_function() for joint in self.get_joints_ordered_by_id(reverse)]

    def get_Xmat_Funcs_ordered_by_name(self, reverse = False):
        return [joint.get_transformation_matrix_function() for joint in self.get_joints_ordered_by_name(reverse)]

    def get_Xmat_Funcs_dict_by_id(self):
        return {joint.jid:joint.get_transformation_matrix_function() for joint in self.joints}

    def get_Xmat_Funcs_dict_by_name(self):
        return {joint.name:joint.get_transformation_matrix_function() for joint in self.joints}

    ##############
    #    IMAT    #
    ##############

    def get_Imat_by_id(self, lid):
        return self.get_link_by_id(lid).get_spatial_inertia()

    def get_Imat_by_name(self, name):
        return self.get_joint_by_name(name).get_spatial_inertia()

    def get_Imats_by_bfs_level(self, level):
        return [link.get_spatial_inertia() for link in self.get_links_by_bfs_level()]

    def get_Imats_ordered_by_id(self, reverse = False):
        return [link.get_spatial_inertia() for link in self.get_links_ordered_by_id(reverse)]

    def get_Imats_ordered_by_name(self, reverse = False):
        return [link.get_spatial_inertia() for link in self.get_links_ordered_by_name(reverse)]

    def get_Imats_dict_by_id(self):
        return {link.lid:link.get_spatial_inertia() for link in self.links}

    def get_Imats_dict_by_name(self):
        return {link.name:link.get_spatial_inertia() for link in self.links}

    ###############
    #      S      #
    ###############

    def get_S_by_id(self, jid):
        return self.get_joint_by_id(jid).get_joint_subspace()

    def get_S_by_name(self, name):
        return self.get_joint_by_name(name).get_joint_subspace()

    def get_S_by_bfs_level(self, level):
        return [joint.get_joint_subspace() for joint in self.get_joints_by_bfs_level()]

    def get_Ss_ordered_by_id(self, reverse = False):
        return [joint.get_joint_subspace() for joint in self.get_joints_ordered_by_id(reverse)]

    def get_Ss_ordered_by_name(self, reverse = False):
        return [joint.get_joint_subspace() for joint in self.get_joints_ordered_by_name(reverse)]

    def get_Ss_dict_by_id(self):
        return {joint.jid:joint.get_joint_subspace() for joint in self.joints}

    def get_Ss_dict_by_name(self):
        return {joint.name:joint.get_joint_subspace() for joint in self.joints}
