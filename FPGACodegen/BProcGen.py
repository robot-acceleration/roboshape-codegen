class BProcGen:
    def __init__(self, num_links, num_PEs, block_size):
        self.num_links = num_links
        self.num_PEs = num_PEs
        self.block_size = block_size
        self.dim_list = ["AX","AY","AZ","LX","LY","LZ"]

    def append_rnea(self, prefix):
        sig = prefix + "_" + "rnea"
        return sig

    def append_dqPE(self, prefix):
        sig_list = []
        for i in range(self.num_PEs):
            sig_name = "{p}_dqPE{i}".format(p=prefix, i=i+1)
            sig_list.append(sig_name)
        return sig_list

    def append_dqdPE(self, prefix):
        sig_list = []
        for i in range(self.num_PEs):
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
                sig_name = "{p}_{dim}_dqPE{i}".format(p=prefix, dim=dim, i=i+1)
                sig_list.append(sig_name)
            sig_list_2d.append(sig_list)
        return sig_list_2d

    def append_dim_dqdPE(self, prefix):
        sig_list_2d = []
        for i in range(self.num_PEs):
            sig_list = []
            for dim in self.dim_list:
                sig_name = "{p}_{dim}_dqdPE{i}".format(p=prefix, dim=dim, i=i+1)
                sig_list.append(sig_name)
            sig_list_2d.append(sig_list)
        return sig_list_2d

    def flatten_2d_list(self, list_2d):
        flattened = [i for sub in list_2d for i in sub]
        return flattened
    
    # radhika: use vecgen to do this after refactoring vw and vecgen libraries
    def genRowColVector_1d(self, prefix):
        sig_list = []
        for i in range(self.num_links):
            for j in range(self.num_links):
                sig_name = "{p}_R{r}_C{c}".format(p=prefix,r=i+1,c=j+1);
                sig_list.append(sig_name)
        return sig_list

    #######################
    #   GEN INPUT PORTS   # 
    #######################

    ### minv_prev_vec_in
    # TODO: deprecated in rev3 with block minv multiply

    def gen_minv_prev_vec_in(self):
        sig_list = []
        for i in range(self.num_links):
            sig_name = "minv_prev_vec_in_C" + str(i+1)
            sig_list.append(sig_name)
        return sig_list

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

    ### f_prev_vec_in

    ## rnea

    def gen_f_prev_vec_in_rnea_ports(self):
        prefix = "f_prev_vec_in"
        return self.append_dim_rnea(prefix)

    ### dfdq_prev_vec_in

    def gen_dfdq_prev_vec_in_dq_ports_by_PE(self):
        prefix = "dfdq_prev_vec_in"
        return self.append_dim_dqPE(prefix)

    def gen_dfdq_prev_vec_in_dq_ports(self):
        list_by_PE_2d = self.gen_dfdq_prev_vec_in_dq_ports_by_PE()
        return self.flatten_2d_list(list_by_PE_2d)
    
    ### dfdqd_prev_vec_in
    
    def gen_dfdqd_prev_vec_in_dqd_ports_by_PE(self):
        prefix = "dfdqd_prev_vec_in"
        return self.append_dim_dqdPE(prefix)

    def gen_dfdqd_prev_vec_in_dqd_ports(self):
        list_by_PE_2d = self.gen_dfdqd_prev_vec_in_dqd_ports_by_PE()
        return self.flatten_2d_list(list_by_PE_2d)

    ### f_upd_curr_vec_in

    ## rnea

    def gen_f_upd_curr_vec_in_rnea_ports(self):
        prefix = "f_upd_curr_vec_in"
        return self.append_dim_rnea(prefix)

    ## dq

    def gen_f_upd_curr_vec_in_dq_ports_by_PE(self):
        prefix = "f_upd_curr_vec_in"
        return self.append_dim_dqPE(prefix)

    def gen_f_upd_curr_vec_in_dq_ports(self):
        list_by_PE_2d = self.gen_f_upd_curr_vec_in_dq_ports_by_PE()
        return self.flatten_2d_list(list_by_PE_2d)


    ### dfdq_upd_curr_vec_in

    def gen_dfdq_upd_curr_vec_in_dq_ports_by_PE(self):
        prefix = "dfdq_upd_curr_vec_in"
        return self.append_dim_dqPE(prefix)

    def gen_dfdq_upd_curr_vec_in_dq_ports(self):
        list_by_PE_2d = self.gen_dfdq_upd_curr_vec_in_dq_ports_by_PE()
        return self.flatten_2d_list(list_by_PE_2d)

    ### dfdqd_upd_curr_vec_in

    def gen_dfdqd_upd_curr_vec_in_dqd_ports_by_PE(self):
        prefix = "dfdqd_upd_curr_vec_in"
        return self.append_dim_dqdPE(prefix)

    def gen_dfdqd_upd_curr_vec_in_dqd_ports(self):
        list_by_PE_2d = self.gen_dfdqd_upd_curr_vec_in_dqd_ports_by_PE()
        return self.flatten_2d_list(list_by_PE_2d)
    
    ### minv_block_in

    def gen_minv_block_in_dqd_ports(self):
        prefix = "minv_block_in"
        sig = prefix + "_R{r}_C{c}_dqdPE{i}"
        sig_list = []
        for i in range(self.num_PEs):
            # signals are named/ordered as transpose of Minv matrix
            for c in range(self.block_size):
                for r in range(self.block_size):
                    sig_list.append(sig.format(r=r+1, c=c+1, i=i+1))
        return sig_list

    ### dtau_vec_in

    def gen_dtau_vec_in_dqd_ports(self):
        prefix = "dtau_vec_in"
        sig = prefix + "_R{r}_dqdPE{i}"
        sig_list = []
        for i in range(self.num_PEs):
            for r in range(self.block_size):
                sig_list.append(sig.format(r=r+1, i=i+1))
        return sig_list


    ########################
    #   GEN OUTPUT PORTS   # 
    ########################

    ### minv_dtaudq_out
    # TODO: deprecated in rev3 with block minv multiply

    def gen_minv_dtaudq_out_ports(self):
        prefix = "minv_dtaudq_out"
        return self.genRowColVector_1d(prefix)

    ### minv_dtaudqd_out
    # TODO: deprecated in rev3 with block minv multiply

    def gen_minv_dtaudqd_out_ports(self):
        prefix = "minv_dtaudqd_out"
        return self.genRowColVector_1d(prefix)

    ### dtau_curr_out

    # dq
    
    def gen_dtau_curr_out_dq_ports(self):
        prefix = "dtau_curr_out"
        return self.append_dqPE(prefix)

    # dqd
    
    def gen_dtau_curr_out_dqd_ports(self):
        prefix = "dtau_curr_out"
        return self.append_dqdPE(prefix)

    ### f_upd_prev_vec_out

    def gen_f_upd_prev_vec_out_rnea_ports(self):
        prefix = "f_upd_prev_vec_out"
        return self.append_dim_rnea(prefix)

    ### dfdq_upd_prev_vec_out

    def gen_dfdq_upd_prev_vec_out_dq_ports_by_PE(self):
        prefix = "dfdq_upd_prev_vec_out"
        return self.append_dim_dqPE(prefix)

    def gen_dfdq_upd_prev_vec_out_dq_ports(self):
        list_by_PE_2d = self.gen_dfdq_upd_prev_vec_out_dq_ports_by_PE()
        return self.flatten_2d_list(list_by_PE_2d)

    ### dfdqd_upd_prev_vec_out

    def gen_dfdqd_upd_prev_vec_out_dqd_ports_by_PE(self):
        prefix = "dfdqd_upd_prev_vec_out"
        return self.append_dim_dqdPE(prefix)

    def gen_dfdqd_upd_prev_vec_out_dqd_ports(self):
        list_by_PE_2d = self.gen_dfdqd_upd_prev_vec_out_dqd_ports_by_PE()
        return self.flatten_2d_list(list_by_PE_2d)

    ### minv_vec_out

    def gen_minv_vec_out_dqd_ports(self):
        prefix = "minv_vec_out"
        sig = prefix + "_R{r}_dqdPE{i}"
        sig_list = []
        for i in range(self.num_PEs):
            for r in range(self.block_size):
                sig_list.append(sig.format(r=r+1, i=i+1))
        return sig_list


    ###===========================================####

    #######################
    #     FILE WRITING    #
    #######################

    ### todo

