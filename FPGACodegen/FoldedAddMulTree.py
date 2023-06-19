import numpy as np
"""
Folds a smaller matrix multiply with a bigger one.
For N < 6:
    smaller = NxN multiply (for Minv)
    larger  = 6x6 multiply (for XT)
For N >= 6:
    smaller = 6x6 multiply
    larger  = NxN multiply
"""
class FoldedAddMulTree:
    def __init__(self, vw, block_size, xform_bools):
        self.block_size = block_size
        self.xform_bools = xform_bools
        self.vw = vw

        if block_size > 6:
            # fold_bool_mat:
            # Entry = True if value fetched from xform matrix
            # rows = block_size, cols = block_size
            #
            # xform_bools    0
            #       0        0
            lower_mat_shape = (block_size - 6, 6)
            lower_mat = np.zeros(lower_mat_shape, dtype=bool)

            right_mat_shape = (block_size, block_size - 6)
            right_mat = np.zeros(right_mat_shape, dtype=bool)

            left_mat = np.vstack((np.array(xform_bools), lower_mat))
            self.xform_nxn_bool_mat = np.hstack((left_mat, right_mat))
        else:
            # fold_bool_mat:
            # Entry = True if value fetched from xform matrix
            # rows = 6, cols = 6
            #
            # fold_bool_mat is just xform_bools, with inner NxN dense matrix
            # marked False
            self.xform_nxn_bool_mat = np.array(xform_bools, dtype=bool)
            for r in range(block_size):
                for c in range(block_size):
                    self.xform_nxn_bool_mat[r,c] = False

    ######################
    #      FOLDING       #
    ######################

    def get_folded_xform_nxn_bool_mat(self):
        return self.xform_nxn_bool_mat

    def gen_folded_signals_list(self, fold_bool_mat, true_signal_arr, false_signal_arr):
        # true_signal_arr should be a regular python arr
        # fold_bool_mat should be a 2D np array
        # false_signal_arr should be a 2D np array
        ### todo: this is confusing, fix

        #assert len(true_signal_arr.shape) == 2, "true only 2D"
        assert len(false_signal_arr.shape) == 2, "false only 2D"
        assert len(fold_bool_mat.shape) == 2, "fold bool only 2D"

        fold_signal_mat = np.empty(shape=fold_bool_mat.shape, dtype=object)

        n_rows = fold_signal_mat.shape[0]
        n_cols = fold_signal_mat.shape[1]

        for r in range(n_rows):
            for c in range(n_cols):
                if fold_bool_mat[r,c] == True:
                    fold_signal_mat[r,c] = true_signal_arr.pop(0)
                else:
                    fold_signal_mat[r,c] = false_signal_arr[r,c]

        return fold_signal_mat.flatten().tolist()

    # Different from gen_folded_signals_list because for N < 6 case,
    # we don't fill up all the False entries from false_signal_array
    def gen_folded_minvm_signals_list(self, xform_signal_arr, minvm_signal_arr,dense_flag=False):
        block_size = self.block_size
        true_signal_arr = xform_signal_arr
        false_signal_arr = minvm_signal_arr

        fold_bool_mat = self.xform_nxn_bool_mat
        fold_signal_mat = None

        if block_size >= 6:
            return self.gen_folded_signals_list(fold_bool_mat, true_signal_arr, false_signal_arr)
        else:
            fold_signal_mat = np.empty(shape=(6,6), dtype=object)
            n_rows = 6
            n_cols = 6

            for r in range(n_rows):
                for c in range(n_cols):
                    if fold_bool_mat[r,c] == True:
                        fold_signal_mat[r,c] = true_signal_arr.pop(0)
                    else:
                        if r < block_size and c < block_size:
                            fold_signal_mat[r,c] = false_signal_arr[r,c]

            fold_signal_list = fold_signal_mat.flatten().tolist()
            if dense_flag == False:
                # for the folded matrix outside minvm, the False values
                # represent the sparse parts of xform_bools
                # so they have no signals associated with them
                # dense_flag is only true for gen_add_mult_tree where
                # we want to transpose matrix of signals, so we keep the None's to preserve shape
                # so if dense_flag is false, we remove all the Nones
                fold_signal_list = [x for x in fold_signal_list if x is not None]
            return fold_signal_list


    def filter_signals_by_bool_mat(self, signals, bool_mat):
        filtered_signals = []

        n_rows = bool_mat.shape[0]
        n_cols = bool_mat.shape[1]

        for i in range(n_rows):
            for j in range(n_rows):
                if bool_mat[i,j] == True:
                    filtered_signals.append(signals[i*n_cols + j])
        return filtered_signals

    def filter_signals_by_xform_bools_6x6(self, dim_signals, xform_bools):
        return self.filter_signals_by_bool_mat(dim_signals, xform_bools)

    def filter_dim_list_signals_by_bool_mat_vanilla(self, prefix, xform_bools):
        filtered_signals = []
        dim_signals = self.vw.gen_dim_signals(prefix)
        filtered_signals = self.filter_signals_by_xform_bools_6x6(dim_signals, xform_bools)
        return filtered_signals

    def filter_dim_list_signals_by_bool_mat_efficient(self, prefix, xform_bools):
        filtered_signals = []
        dim_signals = self.vw.gen_xform_dim_signals(prefix, xform_bools)
        filtered_signals = self.filter_signals_by_xform_bools_6x6(dim_signals, xform_bools)
        return filtered_signals

    ###############################
    #   ADD-MUL TREE GENERATION   #
    ###############################

    # fields for multipliers: unit_name, a_in, b_in, prod_out
    def gen_multipliers_generic(self, op1_sigs, op2_sigs, res_prefix):
        if len(op1_sigs) == 0 or len(op2_sigs) == 0:
            return []
        multiplier_list = []
        associated_vectors = self.vw.associateVectorLists(op1_sigs, op2_sigs)
        for op1, op2 in associated_vectors:
            splitOp1 = op1.split("_")
            s1 = splitOp1[2]
            s2 = splitOp1[3]
            res = "{res_prefix}_{s2}_{s1}".format(res_prefix=res_prefix, s1=s1, s2=s2)
            
            multipler_dict = {}
            multipler_dict["a_in"] = op1
            multipler_dict["b_in"] = op2
            multipler_dict["prod_out"] = res
            multipler_dict["unit_name"] = "mult_{res}".format(res=res)

            multiplier_list.append(multipler_dict)

        return multiplier_list

    # fields for adder tree: 
    # * n_layers = int
    # * n_adders = int
    # * layer0 = list of all the adders in the layer, layer1...
    #
    # fields for adder: unit_name, a_in, b_in, sum_out
    # naming scheme: prefix_s1_s2 + prefix_s1_s3 = prefix_s1_s2s3
    def gen_adder_tree(self, adder_operands):
        adder_tree_dict = {}

        # list of lists
        layers = []
        n_layers = 0
        n_adders = 0

        adder_operand_queue = []
        adder_operand_queue.extend(adder_operands)

        curr_adders_curr_layer = 0
        curr_layer = []
        n_adders_curr_layer = len(adder_operand_queue) // 2

        while len(adder_operand_queue) > 1:
            op1 = adder_operand_queue.pop(0)
            op2 = adder_operand_queue.pop(0)

            splitOp1 = op1.split("_")
            prefix = splitOp1[0]
            s1 = splitOp1[1]
            s2 = splitOp1[2]
            s3 = op2.split("_")[2]
            res = "{prefix}_{s1}_{s2}{s3}".format(prefix=prefix, s1=s1, s2=s2, s3=s3)

            adder_dict = {}
            adder_dict["unit_name"] = "add_{res}".format(res=res)
            adder_dict["a_in"] = op1
            adder_dict["b_in"] = op2
            adder_dict["sum_out"] = res
            curr_layer.append(adder_dict)

            adder_operand_queue.append(res)

            n_adders += 1
            curr_adders_curr_layer += 1

            if curr_adders_curr_layer == n_adders_curr_layer:
                curr_adders_curr_layer = 0
                layers.append(curr_layer)
                curr_layer = []
                n_adders_curr_layer = len(adder_operand_queue) // 2
                n_layers += 1

        adder_tree_dict["n_layers"] = n_layers
        adder_tree_dict["n_adders"] = n_adders
        for i in range(n_layers):
            adder_tree_dict["layer"+str(i)] = layers.pop(0)

        return adder_tree_dict

    def get_mult_output_sigs(self, mult_list):
        output_sigs = []
        for mult in mult_list:
            output_sigs.append(mult["prod_out"])
        return output_sigs

    def get_add_output_sigs(self, add_list):
        output_sigs = []
        for add in add_list:
            output_sigs.append(add["sum_out"])
        return output_sigs

    def get_all_mult_output_sigs(self, mults_per_entry):
        mult_output_signals = []
        for entry in mults_per_entry:
            entry_outputs = self.get_mult_output_sigs(entry)
            mult_output_signals.extend(entry_outputs)
        return mult_output_signals

    def get_add_output_sigs_by_layer(self, tree_per_entry, layer_ind):
        layer_output_signals = []
        for entry in tree_per_entry:
            if layer_ind < entry["n_layers"]:
                entry_outputs = self.get_add_output_sigs(entry["layer"+str(layer_ind)])
                layer_output_signals.extend(entry_outputs)
        return layer_output_signals

    def writeAdderDictListByLayerId(self, tree_per_entry, layer_ind):
        for entry in tree_per_entry:
            if layer_ind < entry["n_layers"]:
                dict_list = entry["layer"+str(layer_ind)]
                self.vw.writeAdderDictList(dict_list)
                self.vw.writeLine("")

    def get_max_layers_across_entries(self, tree_per_entry):
        max_layers = -1
        for entry in tree_per_entry:
            if entry["n_layers"] > max_layers:
                max_layers = entry["n_layers"]
        return max_layers

    def last_layer_output_for_entry(self, tree_per_entry, mults_per_entry, entry_ind):
        entry = tree_per_entry[entry_ind]
        n_layers = entry["n_layers"]
        if n_layers == 0:
            # this means we didn't need to generate any adders for this entry at all
            # entry then must've had <= 1 multiplier and that's it
            # so the last_layer_sig will either be the mult output or NONE
            assert len(mults_per_entry[entry_ind]) <= 1, "sth has gone wrong in add-mul gen"
            if len(mults_per_entry[entry_ind]) == 1:
                return mults_per_entry[entry_ind][0]["prod_out"]
            else:
                return None
        else:
            last_layer_sig = entry["layer"+str(n_layers-1)][0]["sum_out"]
        return last_layer_sig

    def merge_smaller_larger_adder_last_layer(self, smaller_tree_per_entry, larger_tree_per_entry, smaller_mults_per_entry, larger_mults_per_entry):
        last_layer_merged_adders = []

        # the adders only need to be merged for the first `smaller_dim` entries
        # for the rest of the entries, we use the output signals outright
        # because those aren't folded with the smaller multiply.
        if self.block_size >= 6:
            smaller_dim = 6
        else:
            smaller_dim = self.block_size

        for entry_ind in range(smaller_dim):
            smaller_last_layer_output_for_entry = self.last_layer_output_for_entry(smaller_tree_per_entry, smaller_mults_per_entry, entry_ind)
            larger_last_layer_output_for_entry = self.last_layer_output_for_entry(larger_tree_per_entry, larger_mults_per_entry, entry_ind)
            adder_dict = {}
            op1 = smaller_last_layer_output_for_entry
            op2 = larger_last_layer_output_for_entry

            if op1 is None or op2 is None:
                print("heads up, one of the trees is absent, this is a weird corner case so check for errors")
                # assert op1 is not None and op2 is not None, """ corner case where we don't need to merge
                #        two separate minvm and xtvec trees because one wasn't generated at all; didn't cover
                #        this because wanted to avoid changing the output mux generation code for as long as possible.
                #        Need to change it now i guess. RIP."""
                continue

            splitOp1 = op1.split("_")
            prefix = splitOp1[0]
            s1 = splitOp1[1]
            s2 = splitOp1[2]
            s3 = op2.split("_")[2]
            res = "{prefix}_{s1}_{s2}{s3}".format(prefix=prefix, s1=s1, s2=s2, s3=s3)

            adder_dict = {}
            adder_dict["unit_name"] = "add_{res}".format(res=res)
            adder_dict["a_in"] = op1
            adder_dict["b_in"] = op2
            adder_dict["sum_out"] = res

            last_layer_merged_adders.append(adder_dict)

        return last_layer_merged_adders
