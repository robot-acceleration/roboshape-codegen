import numpy as np
from URDFParser import URDFParser
from FPGACodegen import FPGACodegen
from util import VerilogWriter, FileManager
from rbd_config import dim_list, urdf_file, block_size
import time

#---------------------------------------------

parser = URDFParser()
robot = parser.parse(urdf_file)
fpga_codegen = FPGACodegen(robot)

#-------- urdf attrs needed for codegen-ing this file ----------------

num_links = robot.get_num_links_effective()
xform_bools_full = fpga_codegen.get_Xmat_sparsity_boolean_matrix_OR()
xform_bools = fpga_codegen.left_half_matrix(xform_bools_full)

#-------- File management -------------------

fm = FileManager("xtdotminv.v")
output_file_path = fm.get_output_file_path()
xtdotminv_file = open(output_file_path, "w")

vw = VerilogWriter(xtdotminv_file, dim_list, num_links, block_size)

#---------------------------------------------

xtdotminv = fpga_codegen.get_xtdotminv(vw, block_size)

#time.sleep(10)

add_mult_dict = xtdotminv.gen_add_mult_tree()
mults_per_entry = add_mult_dict["mults_per_entry"]
minvm_tree_per_entry = add_mult_dict["minvm_tree_per_entry"]
xtvec_tree_per_entry = add_mult_dict["xtvec_tree_per_entry"]
minvm_mults_per_entry = add_mult_dict["minvm_mults_per_entry"]
xtvec_mults_per_entry = add_mult_dict["xtvec_mults_per_entry"]


print(xform_bools_full)
print(xtdotminv.folded_add_mul_tree.get_folded_xform_nxn_bool_mat())
#vw.print_add_mult_tree(xform_bools_full)

#---------------------------------------------

############

vw.writeLine("`timescale 1ns / 1ps")
vw.writeLine("")
vw.writeLine("// Transformation Matrix Multiplication Transpose by a Vector and Minv")
vw.writeLine("//")
vw.writeLine("// (23 mult, 17 add)")
vw.writeLine("//    AX AY AZ LX LY LZ")
vw.writeLine("// AX  *  *     *  *  *")
vw.writeLine("// AY  *  *  *  *  *")
vw.writeLine("// AZ  *  *  *  *  *")
vw.writeLine("// LX           *  *")
vw.writeLine("// LY           *  *  *")
vw.writeLine("// LZ           *  *  *")
vw.writeLine("")
vw.writeLine("//------------------------------------------------------------------------------")
vw.writeLine("// xtdotminv Module")
vw.writeLine("//------------------------------------------------------------------------------")
vw.writeLine("module xtdotminv#(parameter WIDTH = 32,parameter DECIMAL_BITS = 16)(")
vw.writeLine("   // minv boolean")
vw.writeLine("   input  minv,")

###
vw.writeLine("   // xform_in, 23 values")
vw.writeLine("   input  signed[(WIDTH-1):0]")
vw.setIndentLevel("      ")
vw.writeCommaSep(xtdotminv.gen_xtdotminv_minvm_in_ports(), lineSepElements=xtdotminv.get_larger_dim())
###

###
vw.writeLine("   // vec_in, 6 values")
vw.writeLine("   input  signed[(WIDTH-1):0]")
vw.writeCommaSep(xtdotminv.gen_xtdotminv_vec_in_ports())
###

###
vw.writeLine("   // xtvec_out, 6 values")
vw.writeLine("   output signed[(WIDTH-1):0]")
vw.writeCommaSep(xtdotminv.gen_xtdotminv_vec_out_ports(), last=True)
###

vw.writeLine("   );")
vw.writeLine("")
vw.writeLine("   // internal wires and state")

###
vw.writeLine("   // results from the multiplications")
vw.writeLine("   wire signed[(WIDTH-1):0]")
mult_output_signals = xtdotminv.get_all_mult_output_sigs(mults_per_entry)
vw.writeCommaSep(mult_output_signals, lineSepElements=block_size, last=2)
###

vw.writeLine("")

###
xtvec_max_layers = xtdotminv.get_max_layers_across_entries(xtvec_tree_per_entry)
for i in range(xtvec_max_layers):
    vw.writeLine("   // results from layer {i} of xtvec additions".format(i=i))
    vw.writeLine("   wire signed[(WIDTH-1):0]")
    xtvec_layer_output_sigs = xtdotminv.get_add_output_sigs_by_layer(xtvec_tree_per_entry, i)
    vw.writeCommaSep(xtvec_layer_output_sigs, lineSepElements=3, last=2)
    vw.writeLine("")
###

vw.writeLine("")

###
minvm_max_layers = xtdotminv.get_max_layers_across_entries(minvm_tree_per_entry)
for i in range(minvm_max_layers):
    vw.writeLine("   // results from layer {i} of minvm additions".format(i=i))
    vw.writeLine("   wire signed[(WIDTH-1):0]")
    minvm_layer_output_sigs = xtdotminv.get_add_output_sigs_by_layer(minvm_tree_per_entry, i)
    vw.writeCommaSep(minvm_layer_output_sigs, lineSepElements=3, last=2)
    vw.writeLine("")
###

vw.writeLine("")

###
vw.writeLine("   // multiplications ({linksq} in ||)".format(linksq=block_size**2))
vw.setIndentLevel("   ")
for entry in mults_per_entry:
    vw.writeMultiplierDictList(entry)
    vw.writeLine("")
###

vw.writeLine("")

###
for i in range(xtvec_max_layers):
    vw.writeLine("   // layer {i} of xtvec additions".format(i=i))
    xtdotminv.writeAdderDictListByLayerId(xtvec_tree_per_entry, i)
    vw.writeLine("")
###

vw.writeLine("")

###
for i in range(minvm_max_layers):
    vw.writeLine("   // layer {i} of minvm additions".format(i=i))
    xtdotminv.writeAdderDictListByLayerId(minvm_tree_per_entry, i)
    vw.writeLine("")
###

vw.writeLine("")

###
last_layer_merged_adders = xtdotminv.merge_xtvec_minvm_adder_last_layer(xtvec_tree_per_entry, minvm_tree_per_entry, xtvec_mults_per_entry, minvm_mults_per_entry)
merged_adder_output_signals = []
for adder in last_layer_merged_adders:
    merged_adder_output_signals.append(adder["sum_out"])

# below will be true only if block_size > 6
if merged_adder_output_signals:
    vw.writeLine("   wire signed[(WIDTH-1):0]")
    vw.writeCommaSep(merged_adder_output_signals, last=2)
vw.writeLine("")

vw.writeLine("   // last layer additions merging trees")
vw.writeAdderDictList(last_layer_merged_adders)
###

vw.writeLine("")

###
vw.writeLine("   // mux between last output of OG xtvec tree and merged tree")
if block_size > 6:
    for entry_ind,dim in enumerate(dim_list):
        xtvec_last_layer_output_for_entry = xtdotminv.last_layer_output_for_entry(xtvec_tree_per_entry, xtvec_mults_per_entry, entry_ind)
        minvm_last_layer_output_for_entry = xtdotminv.last_layer_output_for_entry(minvm_tree_per_entry, minvm_mults_per_entry, entry_ind)

        if xtvec_last_layer_output_for_entry and minvm_last_layer_output_for_entry:
            merged_tree_output_for_entry = merged_adder_output_signals.pop(0)
            vw.writeLine("   assign xtvec_out_{dim} = (minv) ? {merge_out} : {xtvec};".format(dim=dim,
                                                                                              merge_out=merged_tree_output_for_entry,
                                                                                              xtvec=xtvec_last_layer_output_for_entry))
        else:
            # if one of these is None, then we aren't muxing between a merged tree and an individual tree at all
            if xtvec_last_layer_output_for_entry is None:
                # xtvec was entirely sparse, no xtvec tree exists
                # so we simply mux between the minvm tree and zero
                xtvec_last_layer_output_for_entry = "32'd0"
            if minvm_last_layer_output_for_entry is None:
                # this will only happen for N = 6, because then the surplus elements
                # which are always minvm, don't exist.
                # This means xtvec was entirely dense, so minvm will be = xtvec
                minvm_last_layer_output_for_entry = xtvec_last_layer_output_for_entry
            vw.writeLine("   assign xtvec_out_{dim} = (minv) ? {minvm} : {xtvec};".format(dim=dim,
                                                                                          minvm=minvm_last_layer_output_for_entry,
                                                                                          xtvec=xtvec_last_layer_output_for_entry))
    for entry_ind in range(6, block_size):
        minvm_last_layer_output_for_entry = xtdotminv.last_layer_output_for_entry(minvm_tree_per_entry, minvm_mults_per_entry, entry_ind)
        vw.writeLine("   assign xtvec_out_C{entry_ind_incr} = (minv) ? {minvm} : 32'd0;".format(entry_ind_incr=entry_ind+1, minvm=minvm_last_layer_output_for_entry))
else:
    for entry_ind,dim in enumerate(dim_list):
        xtvec_last_layer_output_for_entry = xtdotminv.last_layer_output_for_entry(xtvec_tree_per_entry, xtvec_mults_per_entry, entry_ind)
        minvm_last_layer_output_for_entry = xtdotminv.last_layer_output_for_entry(minvm_tree_per_entry, minvm_mults_per_entry, entry_ind)

        if entry_ind < block_size:
            assert minvm_last_layer_output_for_entry is not None, \
                    "minvm layer layer will always exist because inner NxN matrix will always contribute to entry"

        if xtvec_last_layer_output_for_entry and minvm_last_layer_output_for_entry:
            merged_tree_output_for_entry = merged_adder_output_signals.pop(0)
            vw.writeLine("   assign xtvec_out_{dim} = (minv) ? {minvm} : {merge_out};".format(dim=dim,
                                                                                              minvm=minvm_last_layer_output_for_entry,
                                                                                              merge_out=merged_tree_output_for_entry))
        else:
            if not minvm_last_layer_output_for_entry:
                assert entry_ind >= block_size, "there cannot be any minvm tree for anything outside the inner NxN matrix"
                minvm_last_layer_output_for_entry = "32'd0"

            if not xtvec_last_layer_output_for_entry:
                # outer part of xtvec was entirely sparse, no outer xtvec tree exists
                # no mux needed, entry is just minvm tree results
                vw.writeLine("   assign xtvec_out_{dim} = {minvm};".format(dim=dim,
                                                                            minvm=minvm_last_layer_output_for_entry))
            else:
                vw.writeLine("   assign xtvec_out_{dim} = (minv) ? {minvm} : {xtvec};".format(dim=dim,
                                                                                              minvm=minvm_last_layer_output_for_entry,
                                                                                              xtvec=xtvec_last_layer_output_for_entry))
###

vw.writeLine("")
vw.writeLine("endmodule")
