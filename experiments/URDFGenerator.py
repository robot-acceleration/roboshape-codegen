import numpy as np
import os
import random

class URDFGenerator:
    def __init__(self, config, n_links, segment_lengths=[], filepath=""):
        self.n_links = n_links
        self.config = config
        self.segment_lengths = segment_lengths
        self.filename = ""
        self.urdf_out = None

        assert config == "chain" or config == "fork" or config == "torso" or \
                config == "quadruped" or config == "quadruped_manip" or config == "arm_3_fingers", "only chain, fork, torso, quadruped, tine supported"

        if config == "chain":
            self.filename = os.path.join(filepath, "{config}_{links}.urdf".format(config=config, links=n_links))
            self.urdf_out = open(self.filename, 'w')
        else:
            assert sum(segment_lengths) == n_links, "sum of segment lengths doesn't match number of links"

            segment_suffix = "_".join(str(seg) for seg in segment_lengths)
            self.filename = os.path.join(filepath, "{config}_{links}_{suffix}.urdf".format(config=config, links=n_links, suffix=segment_suffix))
            self.urdf_out = open(self.filename, 'w')

    def replace_with_rand_float(self, rand_fmt_str, lower_bound, upper_bound):
        # count number of rands to be generated
        num_rands = rand_fmt_str.count("{f}")

        for i in range(num_rands):
            rand_float = round(random.uniform(lower_bound, upper_bound), 4)
            rand_fmt_str = rand_fmt_str.replace("{f}", str(rand_float), 1)
        return rand_fmt_str

    def replace_with_rand_rotation(self, rand_fmt_str):
        # count number of rands to be generated
        num_rands = rand_fmt_str.count("{r}")

        pi = np.pi
        rot_choices = [-pi, -pi/2, 0, pi/2, pi]

        for i in range(num_rands):
            rand_rot = random.choice(rot_choices)
            rand_fmt_str = rand_fmt_str.replace("{r}", str(rand_rot), 1)
        return rand_fmt_str

    def write_prologue(self):
        urdf_out = self.urdf_out
        urdf_out.write('<?xml version="1.0" encoding="utf-8"?>\n')
        urdf_out.write('<robot name="iiwa">\n')
        urdf_out.write('  <link name="world"/>\n')

    def write_epilogue(self):
        self.urdf_out.write('</robot>\n')

    def generate_chain(self, link_name_fmt_str, joint_name_fmt_str, start_ind, chain_length, parent_link):
        urdf_out = self.urdf_out
        for i in range(start_ind, start_ind+chain_length):
            curr_link_name = link_name_fmt_str.format(l=i)
            urdf_out.write('  <link name="{link_name}">\n'.format(link_name=curr_link_name))
            urdf_out.write('    <inertial>\n')

            # generating random origin xyz, inertia values for link
            rand_link_origin = '      <origin rpy="0 0 0" xyz="{f} {f} {f}"/>\n'
            rand_link_origin = self.replace_with_rand_float(rand_link_origin, 0, 0.1)
            rand_link_inertia = '      <inertia ixx="{f}" ixy="{f}" ixz="{f}" iyy="{f}" iyz="{f}" izz="{f}"/>\n'
            rand_link_inertia = self.replace_with_rand_float(rand_link_inertia, -0.1, 0.1)

            urdf_out.write(rand_link_origin)
            urdf_out.write('      <mass value="5"/>\n')
            urdf_out.write(rand_link_inertia)
            urdf_out.write('    </inertial>\n')
            urdf_out.write('  </link>\n')

            joint_name = joint_name_fmt_str.format(j=i)
            urdf_out.write('  <joint name="{joint_name}" type="revolute">\n'.format(joint_name=joint_name))
            urdf_out.write('    <parent link="{parent}"/>\n'.format(parent=parent_link))
            urdf_out.write('    <child link="{current}"/>\n'.format(current=curr_link_name))

            # generating random origin rpy xyz for joint
            rand_joint_origin = '    <origin rpy="{r} {r} {r}" xyz="0 0 {f}"/>\n'
            rand_joint_origin = self.replace_with_rand_rotation(rand_joint_origin)
            rand_joint_origin = self.replace_with_rand_float(rand_joint_origin, 0, 0.2)

            urdf_out.write(rand_joint_origin)
            urdf_out.write('    <axis xyz="0 0 1"/>\n')
            urdf_out.write('    <limit effort="300" lower="-2.09439510239" upper="2.09439510239" velocity="10"/>\n')
            urdf_out.write('    <dynamics damping="0.5"/>\n')
            urdf_out.write('  </joint>\n')

            parent_link = curr_link_name

    def generate_Y(self):
        segment_lengths = self.segment_lengths
        self.generate_chain("link_{l}", "joint_{j}", 1, segment_lengths[0], "world")

        fork_link_name = "link_" + str(segment_lengths[0])
        self.urdf_out.write('  <!-- ======== LEFT BRANCH STARTS ======= -->\n')
        self.generate_chain("link_{l}_left", "joint_{j}_left", segment_lengths[0]+1, segment_lengths[1], fork_link_name)
        self.urdf_out.write('  <!-- ======== LEFT BRANCH END ======= -->\n')
        self.urdf_out.write('  <!-- ======== RIGHT BRANCH STARTS ======= -->\n')
        self.generate_chain("link_{l}_right", "joint_{j}_right", segment_lengths[0]+1, segment_lengths[2], fork_link_name)
        self.urdf_out.write('  <!-- ======== RIGHT BRANCH ENDS ======= -->\n')

    def generate_torso(self):
        segment_lengths = self.segment_lengths

        self.generate_chain("link_{l}", "joint_{j}", 1, segment_lengths[0], "world")

        curr_start_ind = segment_lengths[0]+1
        for limb_id in range(1,3):
            self.generate_chain("link_{l}", "joint_{j}", curr_start_ind, segment_lengths[limb_id], "world")
            curr_start_ind += segment_lengths[limb_id]

    def generate_quadruped(self):
        segment_lengths = self.segment_lengths

        curr_start_ind = 1
        for limb_id in range(4):
            self.generate_chain("link_{l}", "joint_{j}", curr_start_ind, segment_lengths[limb_id], "world")
            curr_start_ind += segment_lengths[limb_id]

    def generate_quadruped_manip(self):
        segment_lengths = self.segment_lengths

        self.generate_chain("link_{l}", "joint_{j}", 1, segment_lengths[0], "world")

        curr_start_ind = segment_lengths[0]+1
        for limb_id in range(1,5):
            self.generate_chain("link_{l}", "joint_{j}", curr_start_ind, segment_lengths[limb_id], "world")
            curr_start_ind += segment_lengths[limb_id]

    def generate_arm_3_fingers(self):
        segment_lengths = self.segment_lengths

        self.generate_chain("link_{l}", "joint_{j}", 1, segment_lengths[0], "world")

        fork_link_name = "link_" + str(segment_lengths[0])

        curr_start_ind = segment_lengths[0]+1
        for tine_id in range(1,4):
            self.generate_chain("link_{l}", "joint_{j}", curr_start_ind, segment_lengths[tine_id], fork_link_name)
            curr_start_ind += segment_lengths[tine_id]

    def generate_file(self):
        self.write_prologue()
        if self.config == "chain":
            self.generate_chain("link_{l}", "joint_{j}", 1, self.n_links, "world")
        elif self.config == "fork":
            self.generate_Y()
        elif self.config == "torso":
            self.generate_torso()
        elif self.config == "quadruped":
            self.generate_quadruped()
        elif self.config == "quadruped_manip":
            self.generate_quadruped_manip()
        elif self.config == "arm_3_fingers":
            self.generate_arm_3_fingers()
        self.write_epilogue()
        self.urdf_out.close()
