class FProcGen:
    def __init__(self, num_links, num_PEs):
        self.num_links = num_links
        self.num_PEs = num_PEs
        self.dim_list = ["AX","AY","AZ","LX","LY","LZ"]

    def append_rnea(self, prefix):
        sig = prefix + "_" + "rnea"
        return sig

    def append_dqPE(self, prefix):
        sig_list = []
        for i in range(self.num_PEs):
            # TODO: change line below based on
            # sabrina's naming convention
            sig_name = "{p}_dqPE{i}".format(p=prefix, i=i+1)
            sig_list.append(sig_name)
        return sig_list

    def append_dqdPE(self, prefix):
        sig_list = []
        for i in range(self.num_PEs):
            # TODO: change line below based on
            # sabrina's naming convention
            sig_name = "{p}_dqdPE{i}".format(p=prefix, i=i+1)
            sig_list.append(sig_name)
        return sig_list

    def append_dim_rnea(self, prefix):
        sig_list = []
        for dim in self.dim_list:
            sig_name = "{p}_{dim}_rnea".format(p=prefix, dim=dim)
            sig_list.append(sig_name)
        return sig_list

    def append_dim_dqPE(self, prefix):
        sig_list_2d = []
        for i in range(self.num_PEs):
            sig_list = []
            for dim in self.dim_list:
                # TODO: change line below based on
                # sabrina's naming convention
                sig_name = "{p}_{dim}_dqPE{i}".format(p=prefix, dim=dim, i=i+1)
                sig_list.append(sig_name)
            sig_list_2d.append(sig_list)
        return sig_list_2d

    def append_dim_dqdPE(self, prefix):
        sig_list_2d = []
        for i in range(self.num_PEs):
            sig_list = []
            for dim in self.dim_list:
                # TODO: change line below based on
                # sabrina's naming convention
                sig_name = "{p}_{dim}_dqdPE{i}".format(p=prefix, dim=dim, i=i+1)
                sig_list.append(sig_name)
            sig_list_2d.append(sig_list)
        return sig_list_2d

    def flatten_2d_list(self, list_2d):
        flattened = [i for sub in list_2d for i in sub]
        return flattened

    #######################
    #   GEN INPUT PORTS   # 
    #######################

    ### link_in

    def gen_link_in_dq_ports(self):
        port_name = "link_in"
        return self.append_dqPE(port_name)

    def gen_link_in_dqd_ports(self):
        port_name = "link_in"
        return self.append_dqdPE(port_name)

    ### derv_in

    def gen_derv_in_dq_ports(self):
        port_name = "derv_in"
        return self.append_dqPE(port_name)

    def gen_derv_in_dqd_ports(self):
        port_name = "derv_in"
        return self.append_dqdPE(port_name)

    ### sinq_val_in

    def gen_sinq_val_in_dq_ports(self):
        port_name = "sinq_val_in"
        return self.append_dqPE(port_name)

    def gen_sinq_val_in_dqd_ports(self):
        port_name = "sinq_val_in"
        return self.append_dqdPE(port_name)

    ### cosq_val_in

    def gen_cosq_val_in_dq_ports(self):
        port_name = "cosq_val_in"
        return self.append_dqPE(port_name)

    def gen_cosq_val_in_dqd_ports(self):
        port_name = "cosq_val_in"
        return self.append_dqdPE(port_name)

    ### qd_val_in

    def gen_qd_val_in_dq_ports(self):
        port_name = "qd_val_in"
        return self.append_dqPE(port_name)

    def gen_qd_val_in_dqd_ports(self):
        port_name = "qd_val_in"
        return self.append_dqdPE(port_name)

    ### v_curr_vec_in

    ## dq

    def gen_v_curr_vec_in_dq_ports_by_PE(self):
        prefix = "v_curr_vec_in"
        return self.append_dim_dqPE(prefix)

    def gen_v_curr_vec_in_dq_ports(self):
        list_by_PE_2d = self.gen_v_curr_vec_in_dq_ports_by_PE()
        return self.flatten_2d_list(list_by_PE_2d)

    ## dqd

    def gen_v_curr_vec_in_dqd_ports_by_PE(self):
        prefix = "v_curr_vec_in"
        return self.append_dim_dqdPE(prefix)

    def gen_v_curr_vec_in_dqd_ports(self):
        list_by_PE_2d = self.gen_v_curr_vec_in_dqd_ports_by_PE()
        return self.flatten_2d_list(list_by_PE_2d)

    ### a_curr_vec_in

    ## dq

    def gen_a_curr_vec_in_dq_ports_by_PE(self):
        prefix = "a_curr_vec_in"
        return self.append_dim_dqPE(prefix)

    def gen_a_curr_vec_in_dq_ports(self):
        list_by_PE_2d = self.gen_a_curr_vec_in_dq_ports_by_PE()
        return self.flatten_2d_list(list_by_PE_2d)

    ## dqd

    def gen_a_curr_vec_in_dqd_ports_by_PE(self):
        prefix = "a_curr_vec_in"
        return self.append_dim_dqdPE(prefix)

    def gen_a_curr_vec_in_dqd_ports(self):
        list_by_PE_2d = self.gen_a_curr_vec_in_dqd_ports_by_PE()
        return self.flatten_2d_list(list_by_PE_2d)

    ### v_prev_vec_in

    ## rnea

    def gen_v_prev_vec_in_rnea_ports(self):
        prefix = "v_prev_vec_in"
        return self.append_dim_rnea(prefix)

    ## dq

    def gen_v_prev_vec_in_dq_ports_by_PE(self):
        prefix = "v_prev_vec_in"
        return self.append_dim_dqPE(prefix)

    def gen_v_prev_vec_in_dq_ports(self):
        list_by_PE_2d = self.gen_v_prev_vec_in_dq_ports_by_PE()
        return self.flatten_2d_list(list_by_PE_2d)

    ### a_prev_vec_in

    # rnea

    def gen_a_prev_vec_in_rnea_ports(self):
        prefix = "a_prev_vec_in"
        return self.append_dim_rnea(prefix)

    ## dq

    def gen_a_prev_vec_in_dq_ports_by_PE(self):
        prefix = "a_prev_vec_in"
        return self.append_dim_dqPE(prefix)

    def gen_a_prev_vec_in_dq_ports(self):
        list_by_PE_2d = self.gen_a_prev_vec_in_dq_ports_by_PE()
        return self.flatten_2d_list(list_by_PE_2d)

    ###========= EXTERNAL BRANCHING ==============####

    ### dvdq_prev_vec_in

    def gen_dvdq_prev_vec_in_dq_ports_by_PE(self):
        prefix = "dvdq_prev_vec_in"
        return self.append_dim_dqPE(prefix)

    def gen_dvdq_prev_vec_in_dq_ports(self):
        list_by_PE_2d = self.gen_dvdq_prev_vec_in_dq_ports_by_PE()
        return self.flatten_2d_list(list_by_PE_2d)
    
    ### dvdqd_prev_vec_in
    
    def gen_dvdqd_prev_vec_in_dqd_ports_by_PE(self):
        prefix = "dvdqd_prev_vec_in"
        return self.append_dim_dqdPE(prefix)

    def gen_dvdqd_prev_vec_in_dqd_ports(self):
        list_by_PE_2d = self.gen_dvdqd_prev_vec_in_dqd_ports_by_PE()
        return self.flatten_2d_list(list_by_PE_2d)

    ### dadq_prev_vec_in

    def gen_dadq_prev_vec_in_dq_ports_by_PE(self):
        prefix = "dadq_prev_vec_in"
        return self.append_dim_dqPE(prefix)

    def gen_dadq_prev_vec_in_dq_ports(self):
        list_by_PE_2d = self.gen_dadq_prev_vec_in_dq_ports_by_PE()
        return self.flatten_2d_list(list_by_PE_2d)
    
    ### dadqd_prev_vec_in
    
    def gen_dadqd_prev_vec_in_dqd_ports_by_PE(self):
        prefix = "dadqd_prev_vec_in"
        return self.append_dim_dqdPE(prefix)

    def gen_dadqd_prev_vec_in_dqd_ports(self):
        list_by_PE_2d = self.gen_dadqd_prev_vec_in_dqd_ports_by_PE()
        return self.flatten_2d_list(list_by_PE_2d)


    ###===========================================####

    ########################
    #   GEN OUTPUT PORTS   # 
    ########################

    ### v_curr_vec_out

    def gen_v_curr_vec_out_rnea_ports(self):
        prefix = "v_curr_vec_out"
        return self.append_dim_rnea(prefix)

    ### a_curr_vec_out

    def gen_a_curr_vec_out_rnea_ports(self):
        prefix = "a_curr_vec_out"
        return self.append_dim_rnea(prefix)

    ### f_curr_vec_out

    def gen_f_curr_vec_out_rnea_ports(self):
        prefix = "f_curr_vec_out"
        return self.append_dim_rnea(prefix)

    ### dfdq_curr_vec_out

    ## dq

    def gen_dfdq_curr_vec_out_dq_ports_by_PE(self):
        port_name = "dfdq_curr_vec_out"
        return self.append_dim_dqPE(port_name)

    def gen_dfdq_curr_vec_out_dq_ports(self):
        list_by_PE_2d = self.gen_dfdq_curr_vec_out_dq_ports_by_PE()
        return self.flatten_2d_list(list_by_PE_2d)

    ### dfdqd_curr_vec_out

    # dqd

    def gen_dfdqd_curr_vec_out_dqd_ports_by_PE(self):
        port_name = "dfdqd_curr_vec_out"
        return self.append_dim_dqdPE(port_name)

    def gen_dfdqd_curr_vec_out_dqd_ports(self):
        list_by_PE_2d = self.gen_dfdqd_curr_vec_out_dqd_ports_by_PE()
        return self.flatten_2d_list(list_by_PE_2d)

    ###========= EXTERNAL BRANCHING ==============####

    ### dvdq_curr_vec_out

    def gen_dvdq_curr_vec_out_dq_ports_by_PE(self):
        prefix = "dvdq_curr_vec_out"
        return self.append_dim_dqPE(prefix)

    def gen_dvdq_curr_vec_out_dq_ports(self):
        list_by_PE_2d = self.gen_dvdq_curr_vec_out_dq_ports_by_PE()
        return self.flatten_2d_list(list_by_PE_2d)
    
    ### dvdqd_curr_vec_out
    
    def gen_dvdqd_curr_vec_out_dqd_ports_by_PE(self):
        prefix = "dvdqd_curr_vec_out"
        return self.append_dim_dqdPE(prefix)

    def gen_dvdqd_curr_vec_out_dqd_ports(self):
        list_by_PE_2d = self.gen_dvdqd_curr_vec_out_dqd_ports_by_PE()
        return self.flatten_2d_list(list_by_PE_2d)

    ### dadq_curr_vec_out

    def gen_dadq_curr_vec_out_dq_ports_by_PE(self):
        prefix = "dadq_curr_vec_out"
        return self.append_dim_dqPE(prefix)

    def gen_dadq_curr_vec_out_dq_ports(self):
        list_by_PE_2d = self.gen_dadq_curr_vec_out_dq_ports_by_PE()
        return self.flatten_2d_list(list_by_PE_2d)
    
    ### dadqd_curr_vec_out
    
    def gen_dadqd_curr_vec_out_dqd_ports_by_PE(self):
        prefix = "dadqd_curr_vec_out"
        return self.append_dim_dqdPE(prefix)

    def gen_dadqd_curr_vec_out_dqd_ports(self):
        list_by_PE_2d = self.gen_dadqd_curr_vec_out_dqd_ports_by_PE()
        return self.flatten_2d_list(list_by_PE_2d)


    ###===========================================####

    #######################
    #     FILE WRITING    #
    #######################

    ### todo
