from URDFParser import URDFParser
from FPGACodegen import FPGACodegen
from util import VerilogWriter, FileManager
from rbd_config import dim_list, urdf_file

#---------------------------------------------

parser = URDFParser()
robot = parser.parse(urdf_file)
fpga_codegen = FPGACodegen(robot)

#-------- urdf attrs needed for codegen-ing this file ----------------

num_joints = robot.get_num_joints()
xform_bools_full = fpga_codegen.get_Xmat_sparsity_boolean_matrix_OR()
xform_bools = fpga_codegen.left_half_matrix(xform_bools_full)

#-------- File management -------------------

fm = FileManager("xgens{num_joints}.v".format(num_joints=num_joints))
output_file_path = fm.get_output_file_path()
xgens_file = open(output_file_path, "w")

vw = VerilogWriter(xgens_file, dim_list, num_joints)

#------------ Bitwidth stuff -----------------

# Find Bitwidth of # Links
bitwidth_num_joints = vw.get_bitwidth(num_joints)
bitwidth_num_joints_str = str(bitwidth_num_joints)

#---------------------------------------------

vw.writeLine("`timescale 1ns / 1ps")
vw.writeLine("")
vw.writeLine("// Transformation Matrix Generation")
vw.writeLine("")
vw.writeLine("//------------------------------------------------------------------------------")
vw.writeLine("// xgens{num_joints} Module".format(num_joints=num_joints))
vw.writeLine("//------------------------------------------------------------------------------")
vw.writeLine("module xgens{num_joints}#(parameter WIDTH = 32,parameter DECIMAL_BITS = 16)(".format(num_joints=num_joints))
vw.writeLine("   // link_in")
vw.writeLine("   input  [" + str(vw.get_bitwidth_array_def(num_joints)) + ":0]")
vw.writeLine("      link_in,")
vw.writeLine("   // sin(q) and cos(q)")
vw.writeLine("   input  signed[(WIDTH-1):0]")
vw.writeLine("      sinq_in,cosq_in,")
vw.writeLine("")

###
vw.writeLine("   // internal wires and state")
mat_str = vw.makeXformOutputPort(xform_bools)
xgens_file.write(mat_str)
###

vw.writeLine("   );")
vw.writeLine("")
vw.writeLine("   // internal wires and state")
vw.writeLine("")

###
for i in range(1, num_joints+1):
    wire_prefix = "xform_l" + str(i)+ "_out_"
    mat_str = vw.makeXformWiresPrefix(xform_bools,wire_prefix)
    xgens_file.write(mat_str)
    vw.writeLine()
###

vw.writeLine("")

###
for i in range(1, num_joints+1):
    vw.setIndentLevel("   ")
    vw.writeLine("   // Link = {i}".format(i=i))
    vw.writeLine("   xgen{p}X{c}#(.WIDTH(WIDTH),.DECIMAL_BITS(DECIMAL_BITS))".format(p=i, c=i-1))
    vw.setIndentLevel("      ")
    vw.writeLine("      uut{p}X{c}(".format(p=i, c=i-1))
    vw.writeLine("      // sin(q) and cos(q)")
    vw.writeLine("      .sinq_in(sinq_in),.cosq_in(cosq_in),")
    vw.writeLine("      // xform_out, 15 values")
    ###
    wire_prefix = "xform_l" + str(i)+ "_out_"
    mat_str = vw.makeXformXgensPortAssignmentsWirePrefix(xform_bools,wire_prefix)
    xgens_file.write(mat_str)
    vw.writeLine("      );")
    ###
vw.writeLine("")
###

vw.writeLine("   // output muxes")
###
xform_dims = []
for row,row_d6 in enumerate(dim_list):
    for col in range(0,3):
        col_d6 = dim_list[col]
        xfm_bool = xform_bools[row][col]
        if (xfm_bool):
            dim = row_d6+"_"+col_d6
            xform_dims.append(dim)

for dim in xform_dims:
    vw.writeLine("// {dim}".format(dim=dim))
    mux_str = "   assign xform_out_{dim} = ".format(dim=dim)
    for j in range(1, num_joints+1):
        mux_str += "(link_in == {bitwidth}'d{j}) ? xform_l{j}_out_{dim} :\n".format(bitwidth=bitwidth_num_joints_str, j=j, dim=dim)
        mux_str += "                            "
    mux_str += "32'd0;\n"

    xgens_file.write(mux_str)
###

vw.writeLine("")
vw.writeLine("endmodule")
