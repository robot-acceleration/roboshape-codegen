from .FoldedAddMulTree import FoldedAddMulTree
import numpy as np

class XtDotMinvGen:
    def __init__(self, vw, block_size, xform_bools):
        self.block_size = block_size
        self.xform_bools = xform_bools
        self.folded_add_mul_tree = FoldedAddMulTree(vw, block_size, xform_bools)
        self.vw = vw

        if block_size >= 6:
            self.larger_dim = block_size
        else:
            self.larger_dim = 6
            # storing actual xform_bools for setting up input muxes to xtdotminv mat input
            self.inner_xform_bools = self.xform_bools

            # this is a hack
            ### TODO: write better explanation
            # tl;dr: 
            # * for N >= 6, we are folding a sparse multiply into a dense one
            # * for N < 6, we are folding a dense multiply into a sparse one
            # the smaller dim tree is dense, the larger one is sparse
            # so we only generate a separate tree for the sparse outer dim
            # and merge it with the dense inner dim
            # |<--- N --->   |
            #/ \         |   |
            # |   dense  |   |
            # N  (false) |   | 6 x 6
            # |          |   |
            #\ /_________|   |
            #  sparse (true) |
            #----------------
            # sparse outer xform signals marked true, everything else false
            # what this really is `outer_xform_bools`
            # we almost always use outer_xform_bools in this file because
            # the sparse add-mul trees are generated only from this
            self.xform_bools = self.folded_add_mul_tree.get_folded_xform_nxn_bool_mat()


    #############
    #  GETTERS  #
    #############

    def get_larger_dim(self):
        return self.larger_dim

    def get_input_xform_bools(self):
        if self.block_size >= 6:
            return self.xform_bools
        else:
            return self.inner_xform_bools

    #######################
    #  GEN INPUT SIGNALS  # 
    #######################

    def gen_xtdotminv_minvm_in(self):
        block_size = self.block_size
        folded_add_mul_tree = self.folded_add_mul_tree
        vw = self.vw
        
        minv_in_1d = vw.genColRowVector_1d_block_size("xtdot_minvm_in")
        dim_list_filtered = folded_add_mul_tree.filter_dim_list_signals_by_bool_mat_vanilla("xtdot_xform_in", self.xform_bools)

        # reshaping
        minv_in_2d = np.reshape(np.array(minv_in_1d, dtype=object), (self.block_size, self.block_size))

        final_input = folded_add_mul_tree.gen_folded_minvm_signals_list(dim_list_filtered, minv_in_2d)
        
        return final_input

    def gen_xtdotminv_vec_in(self):
        block_size = self.block_size
        folded_add_mul_tree = self.folded_add_mul_tree
        vw = self.vw

        dim_list_signals = vw.dimNameFormatStr("xtdot_vec_in_{dim}")
        vec_in_1d = vw.blockSizeFormatStr("xtdot_vec_in_C{b}")

        fold_bool_mat = [True] * 6
        if block_size >= 6:
            fold_bool_mat.extend([False] * (block_size-6))

        # reshaping
        vec_in_2d = np.reshape(np.array(vec_in_1d, dtype=object), (1, self.block_size))
        fold_bool_mat = np.reshape(fold_bool_mat, (1, self.larger_dim))

        final_input = folded_add_mul_tree.gen_folded_signals_list(fold_bool_mat, dim_list_signals, vec_in_2d)
        
        return final_input

    ########################
    #  GEN OUTPUT SIGNALS  # 
    ########################

    def gen_xtdotminv_vec_out(self):
        block_size = self.block_size
        folded_add_mul_tree = self.folded_add_mul_tree
        vw = self.vw

        dim_list_signals = vw.dimNameFormatStr("xtdot_vec_out_{dim}")
        vec_out_1d = vw.blockSizeFormatStr("xtdot_vec_out_C{b}")

        fold_bool_mat = [True] * 6
        if block_size >= 6:
            fold_bool_mat.extend([False] * (block_size-6))

        # reshaping
        vec_out_2d = np.reshape(np.array(vec_out_1d, dtype=object), (1, self.block_size))
        fold_bool_mat = np.reshape(fold_bool_mat, (1, self.larger_dim))

        final_input = folded_add_mul_tree.gen_folded_signals_list(fold_bool_mat, dim_list_signals, vec_out_2d)
        
        return final_input

    #######################
    #  SETUP INPUT MUXES  #
    #######################

    def fill_filtered_dim_list_minvm_in(self):
        block_size = self.block_size
        folded_add_mul_tree = self.folded_add_mul_tree
        vw = self.vw
        input_xform_bools = self.get_input_xform_bools()

        dim_list_filtered = folded_add_mul_tree.filter_dim_list_signals_by_bool_mat_efficient("xform", input_xform_bools)
        
        filled_signal_list = []

        if block_size > 6:
            larger_dim = block_size
            fold_bool_mat = folded_add_mul_tree.get_folded_xform_nxn_bool_mat()
        else:
            larger_dim = 6
            fold_bool_mat = input_xform_bools

        for i in range(larger_dim):
            for j in range(larger_dim):
                if fold_bool_mat[i,j] == True:
                    filled_signal_list.append(dim_list_filtered.pop(0))
                else:
                    if block_size >= 6:
                        filled_signal_list.append("0")
                    else:
                        if i < block_size and j < block_size:
                            filled_signal_list.append("0")
                        # else:
                            # similar to gen_folded_minvm_signals_list()
                            # for folded matrix outside minvm, the False
                            # entries represent sparse parts of xform_bools
                            # so they don't have a minvm or an xform signal
                            # associated with them.
                            # we don't even generate a port for those entries


        return filled_signal_list

    def fill_filtered_dim_list_vec_in(self):
        block_size = self.block_size
        folded_add_mul_tree = self.folded_add_mul_tree
        vw = self.vw

        fold_bool_mat = [True] * 6
        if block_size < 6:
            vec_in_len = 6
        if block_size >= 6:
            fold_bool_mat.extend([False] * (block_size-6))
            vec_in_len = block_size

        dim_list_filtered = vw.dimNameFormatStr("dfdqd_curr_in_{dim}")
        
        filled_signal_list = []

        for i in range(vec_in_len):
            if fold_bool_mat[i] == True:
                filled_signal_list.append(dim_list_filtered.pop(0))
            else:
                filled_signal_list.append("0")

        return filled_signal_list

    def gen_xtdotminv_minvm_in_mux_params(self):
        block_size = self.block_size
        xform_bools = self.xform_bools
        folded_add_mul_tree = self.folded_add_mul_tree
        vw = self.vw

        if block_size >= 6:
            larger_dim = block_size
        else:
            larger_dim = 6

        lhs = self.gen_xtdotminv_minvm_in()
        rhs_if = ["minv"] * len(lhs)
        rhs_true = vw.genColRowVector_1d_block_size("minvm_in")

        if block_size < 6:
            remaining_xform_entries = len(lhs) - len(rhs_true)
            rhs_true.extend(["0"] * remaining_xform_entries)

        rhs_false = self.fill_filtered_dim_list_minvm_in()

        print(len(lhs))
        print(lhs)
        print(len(rhs_if))
        print(rhs_if)
        print(len(rhs_true))
        print(rhs_true)
        print(len(rhs_false))
        print(rhs_false)

        return [lhs, rhs_if, rhs_true, rhs_false]

    def write_xtdotminv_minvm_in_mux(self):
        block_size = self.block_size
        folded_add_mul_tree = self.folded_add_mul_tree
        vw = self.vw

        lhs, rhs_if, rhs_true, rhs_false = self.gen_xtdotminv_minvm_in_mux_params()

        for i in range(len(lhs)):
            vw.assignMux(lhs[i], rhs_if[i], rhs_true[i], rhs_false[i])
            if i % block_size == (block_size-1):
                vw.writeLine("")

    def gen_xtdotminv_vec_in_mux_params(self):
        block_size = self.block_size
        folded_add_mul_tree = self.folded_add_mul_tree
        vw = self.vw

        if block_size >= 6:
            larger_dim = block_size
        else:
            larger_dim = 6

        lhs = self.gen_xtdotminv_vec_in()
        rhs_if = ["minv"] * larger_dim
        rhs_true = vw.blockSizeFormatStr("tau_vec_in_R{b}")

        if block_size < 6:
            remaining_xform_entries = 6 - block_size
            rhs_true.extend(["0"] * remaining_xform_entries)

        rhs_false = self.fill_filtered_dim_list_vec_in()

        return [lhs, rhs_if, rhs_true, rhs_false]

    def write_xtdotminv_vec_in_mux(self):
        block_size = self.block_size
        folded_add_mul_tree = self.folded_add_mul_tree
        vw = self.vw

        lhs, rhs_if, rhs_true, rhs_false = self.gen_xtdotminv_vec_in_mux_params()

        for i in range(len(lhs)):
            vw.assignMux(lhs[i], rhs_if[i], rhs_true[i], rhs_false[i])

    #####################
    #  GEN INPUT PORTS  # 
    #####################

    def gen_xtdotminv_minvm_in_ports(self, dense_flag=False):
        block_size = self.block_size
        xform_bools = self.xform_bools
        folded_add_mul_tree = self.folded_add_mul_tree
        vw = self.vw

        minv_in_1d = vw.genColRowVector_1d_block_size("minvm_in")
        dim_list_filtered = folded_add_mul_tree.filter_dim_list_signals_by_bool_mat_vanilla("xform_in", xform_bools)

        # reshaping
        minv_in_2d = np.reshape(np.array(minv_in_1d, dtype=object), (self.block_size, self.block_size))

        fold_bool_mat = folded_add_mul_tree.get_folded_xform_nxn_bool_mat()

        final_input = folded_add_mul_tree.gen_folded_minvm_signals_list(dim_list_filtered, minv_in_2d, dense_flag)
        
        return final_input

    def write_xtdotminv_minvm_in_port_assignments(self):
        block_size = self.block_size
        folded_add_mul_tree = self.folded_add_mul_tree
        vw = self.vw

        ports = self.gen_xtdotminv_minvm_in_ports()
        signals = self.gen_xtdotminv_minvm_in()
        vw.portAssignmentVectorLists(ports, signals, lineSepElements=self.block_size)

    def gen_xtdotminv_vec_in_ports(self):
        block_size = self.block_size
        folded_add_mul_tree = self.folded_add_mul_tree
        vw = self.vw

        dim_list_signals = vw.dimNameFormatStr("vec_in_{dim}")
        vec_in_1d = vw.blockSizeFormatStr("vec_in_C{b}")

        fold_bool_mat = [True] * 6
        if block_size >= 6:
            fold_bool_mat.extend([False] * (block_size-6))

        # reshaping
        vec_in_2d = np.reshape(np.array(vec_in_1d, dtype=object), (1, self.block_size))
        fold_bool_mat = np.reshape(fold_bool_mat, (1, self.larger_dim))

        final_input = folded_add_mul_tree.gen_folded_signals_list(fold_bool_mat, dim_list_signals, vec_in_2d)
        
        return final_input

    def write_xtdotminv_vec_in_port_assignments(self):
        block_size = self.block_size
        folded_add_mul_tree = self.folded_add_mul_tree
        vw = self.vw

        ports = self.gen_xtdotminv_vec_in_ports()
        signals = self.gen_xtdotminv_vec_in()
        vw.portAssignmentVectorLists(ports, signals, lineSepElements=self.block_size)

    ######################
    #  GEN OUTPUT PORTS  # 
    ######################

    def gen_xtdotminv_vec_out_ports(self):
        block_size = self.block_size
        folded_add_mul_tree = self.folded_add_mul_tree
        vw = self.vw

        dim_list_signals = vw.dimNameFormatStr("xtvec_out_{dim}")
        vec_in_1d = vw.blockSizeFormatStr("xtvec_out_C{b}")

        fold_bool_mat = [True] * 6
        if block_size >= 6:
            fold_bool_mat.extend([False] * (block_size-6))

        # reshaping
        vec_in_2d = np.reshape(np.array(vec_in_1d, dtype=object), (1, self.block_size))
        fold_bool_mat = np.reshape(fold_bool_mat, (1, self.larger_dim))

        final_input = folded_add_mul_tree.gen_folded_signals_list(fold_bool_mat, dim_list_signals, vec_in_2d)
        
        return final_input

    def write_xtdotminv_vec_out_port_assignments(self):
        block_size = self.block_size
        folded_add_mul_tree = self.folded_add_mul_tree
        vw = self.vw

        ports = self.gen_xtdotminv_vec_out_ports()
        signals = self.gen_xtdotminv_vec_out()
        vw.portAssignmentVectorLists(ports, signals, lineSepElements=self.larger_dim, last=True)

    def assign_minv_outputs(self):
        block_size = self.block_size
        vw = self.vw

        lhs = vw.blockSizeFormatStr("minv_vec_out_R{b}")
        rhs = self.gen_xtdotminv_vec_out()
        if block_size < 6:
            rhs = rhs[:block_size]
        vw.assignVectorLists(lhs, rhs)

    #####################
    #  GEN ADDMUL TREE  # 
    #####################

    # fields for multipliers: unit_name, a_in, b_in, prod_out
    # prod_out naming scheme: xform_in_s1_s2 -> xtvec_s2_s1
    def gen_multiplers_xtvec(self, xform_in_sigs, vec_in_sigs):
        return self.folded_add_mul_tree.gen_multipliers_generic(xform_in_sigs, vec_in_sigs, "xtvec")

    # fields for multipliers: unit_name, a_in, b_in, prod_out
    # prod_out naming scheme: minvm_in_s1_s2 -> minvm_s2_s1
    def gen_multiplers_minvm(self, minvm_in_sigs, vec_in_sigs):
        return self.folded_add_mul_tree.gen_multipliers_generic(minvm_in_sigs, vec_in_sigs, "minvm")

    def merge_xtvec_minvm_adder_last_layer(self, xtvec_tree_per_entry, minvm_tree_per_entry, xtvec_mults_per_entry, minvm_mults_per_entry):
        folded_add_mul_tree = self.folded_add_mul_tree

        if self.block_size >= 6:
            return folded_add_mul_tree.merge_smaller_larger_adder_last_layer(xtvec_tree_per_entry, minvm_tree_per_entry, xtvec_mults_per_entry, minvm_mults_per_entry)
        else:
            return folded_add_mul_tree.merge_smaller_larger_adder_last_layer(minvm_tree_per_entry, xtvec_tree_per_entry, minvm_mults_per_entry, xtvec_mults_per_entry)

    def select_signals_by_bool(self, row, row_select_bool):
        selected_signals = []
        for i,sel in enumerate(row_select_bool):
            if sel == True:
                if row[i] is not None:
                    selected_signals.append(row[i])
        return selected_signals

    def gen_add_mult_tree(self):
        block_size = self.block_size
        xform_bools = self.xform_bools
        folded_add_mul_tree = self.folded_add_mul_tree
        larger_dim = self.larger_dim

        # don't be worried about the None's in this list, we filter
        # those out anyway
        minvm_combined_list = self.gen_xtdotminv_minvm_in_ports(dense_flag=True)
        minvm_combined_mat = np.reshape(np.array(minvm_combined_list, dtype=object), (larger_dim, larger_dim))

        fold_bool_mat = folded_add_mul_tree.get_folded_xform_nxn_bool_mat()

        # transposing because we feed the columns as input vectors for mult generation
        minvm_combined_trans = np.transpose(minvm_combined_mat)
        fold_bool_mat_trans = np.transpose(fold_bool_mat)

        vec_in_sigs = self.gen_xtdotminv_vec_in_ports()

        # block_size entries
        mults_per_entry = []
        minvm_mults_per_entry = []
        xtvec_mults_per_entry = []
        minvm_tree_per_entry = []
        xtvec_tree_per_entry = []

        for i in range(larger_dim):
            curr_col_mults = []

            full_row = minvm_combined_trans[i]
            xtvec_select_bool = fold_bool_mat_trans[i]
            xtvec_sigs = self.select_signals_by_bool(full_row, xtvec_select_bool)
            vec_in_operands_xtvec = self.select_signals_by_bool(vec_in_sigs, xtvec_select_bool)

            minvm_select_bool = np.logical_not(fold_bool_mat_trans[i])
            minvm_sigs = self.select_signals_by_bool(full_row, minvm_select_bool)
            vec_in_operands_minvm = self.select_signals_by_bool(vec_in_sigs, minvm_select_bool)

            minvm_mults = self.gen_multiplers_minvm(minvm_sigs, vec_in_operands_minvm)
            xtvec_mults = self.gen_multiplers_xtvec(xtvec_sigs, vec_in_operands_xtvec)

            minvm_mults_per_entry.append(minvm_mults)
            xtvec_mults_per_entry.append(xtvec_mults)

            curr_col_mults.extend(minvm_mults)
            curr_col_mults.extend(xtvec_mults)
            mults_per_entry.append(curr_col_mults)

            minvm_adder_operands = folded_add_mul_tree.get_mult_output_sigs(minvm_mults)
            xtvec_adder_operands = folded_add_mul_tree.get_mult_output_sigs(xtvec_mults)

            minvm_adder_tree = folded_add_mul_tree.gen_adder_tree(minvm_adder_operands)
            xtvec_adder_tree = folded_add_mul_tree.gen_adder_tree(xtvec_adder_operands)

            minvm_tree_per_entry.append(minvm_adder_tree)
            xtvec_tree_per_entry.append(xtvec_adder_tree)

        res_dict = {}
        res_dict["mults_per_entry"] = mults_per_entry
        res_dict["minvm_mults_per_entry"] = minvm_mults_per_entry
        res_dict["xtvec_mults_per_entry"] = xtvec_mults_per_entry
        res_dict["minvm_tree_per_entry"] = minvm_tree_per_entry
        res_dict["xtvec_tree_per_entry"] = xtvec_tree_per_entry

        return res_dict

    def print_mult_dict(self, list_per_entry):
        for comp_list in list_per_entry:
            for comp in comp_list:
                print(comp)
            print()

    def print_add_tree(self, add_tree):
        for e,entry in enumerate(add_tree):
            print("Entry: " + str(e))
            n_layers = entry["n_layers"]
            for i in range(n_layers):
                print(entry["layer"+str(i)])
            print()

    def print_add_mult_tree(self):
        add_mult_dict = self.gen_add_mult_tree(xform_bools)
        mults_per_entry = add_mult_dict["mults_per_entry"]
        minvm_tree_per_entry = add_mult_dict["minvm_tree_per_entry"]
        xtvec_tree_per_entry = add_mult_dict["xtvec_tree_per_entry"]

        #self.print_mult_dict(mults_per_entry)
        #self.print_add_tree(minvm_tree_per_entry)
        #self.print_add_tree(xtvec_tree_per_entry)

    #############################
    # FoldedAddMulTree WRAPPERS #
    #############################

    def get_all_mult_output_sigs(self, mult_list):
        return self.folded_add_mul_tree.get_all_mult_output_sigs(mult_list)

    def get_max_layers_across_entries(self, tree_per_entry):
        return self.folded_add_mul_tree.get_max_layers_across_entries(tree_per_entry)

    def get_add_output_sigs_by_layer(self, tree_per_entry, layer_ind):
        return self.folded_add_mul_tree.get_add_output_sigs_by_layer(tree_per_entry, layer_ind)

    def last_layer_output_for_entry(self, tree_per_entry, mults_per_entry, entry_ind):
        return self.folded_add_mul_tree.last_layer_output_for_entry(tree_per_entry, mults_per_entry, entry_ind)

    def writeAdderDictListByLayerId(self, tree_per_entry, layer_ind):
        return self.folded_add_mul_tree.writeAdderDictListByLayerId(tree_per_entry, layer_ind)

