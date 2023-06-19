import sympy as sp
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

#---------------------------------------------

for i in range(num_joints):
    #-------- File management -------------------

    fm = FileManager("xgen{p}X{c}.v".format(p=i+1, c=i))
    output_file_path = fm.get_output_file_path()
    xgen_file = open(output_file_path, "w")

    vw = VerilogWriter(xgen_file, dim_list, num_joints)

    #---------------------------------------------

    Xmat = robot.get_Xmat_by_id(i)

    #---------------------------------------------

    vw.writeLine("`timescale 1ns / 1ps")
    vw.writeLine("")
    vw.writeLine("// Transformation Matrix Generation")
    vw.writeLine("")
    vw.writeLine("//------------------------------------------------------------------------------")
    vw.writeLine("// xgen Module for Link 1")
    vw.writeLine("//------------------------------------------------------------------------------")
    vw.writeLine("module xgen{p}X{c}#(parameter WIDTH = 32,parameter DECIMAL_BITS = 16)(".format(p=i+1, c=i))
    vw.writeLine("   // sin(q) and cos(q)")
    vw.writeLine("   input  signed[(WIDTH-1):0]")
    vw.writeLine("      sinq_in,cosq_in,")

    ###
    vw.writeLine("   // xform_out, 15 values")
    mat_str = vw.makeXformOutputPort(xform_bools)
    xgen_file.write(mat_str)
    ###

    vw.writeLine("   );")
    vw.writeLine("")

    vw.writeLine("   // internal wires and state")
    vw.writeLine("   wire signed[(WIDTH-1):0]")
    vw.writeLine("      sinq,cosq,nsinq,ncosq;")
    ###
    vw.setIndentLevel("      ")
    mat_str = vw.makeXformWires(xform_bools)
    xgen_file.write(mat_str)
    vw.writeLine()
    ###

    ###
    # looping over left half of Xmat
    for r in range(6):
        for c in range(3):
            entry = Xmat[r,c]
            if xform_bools[r][c] == True and entry.is_real:
                cst_str = fpga_codegen.to_decimal_string_repr(entry)
                vw.writeLine("   assign xform_{rdim}_{cdim} = {cst_str};".format(rdim=dim_list[r], cdim=dim_list[c], cst_str=cst_str))
    ###

    vw.writeLine("")
    vw.writeLine("   // variables")
    vw.writeLine("   assign sinq  = sinq_in;")
    vw.writeLine("   assign nsinq = -sinq_in;")
    vw.writeLine("   assign cosq  = cosq_in;")
    vw.writeLine("   assign ncosq  = -cosq_in;")
    ###
    vw.writeLine("   // ---")
    for r in range(6):
        for c in range(3):
            val = Xmat[r,c]
            val_str = str(val)
            trig_str = None
            # bad hack to check for sines and cosines
            if val_str[0:3] == "sin":
                trig_str = "sinq"
            elif val_str[0:3] == "cos":
                trig_str = "cosq"
            elif val_str[0:4] == "-sin":
                trig_str = "nsinq"
            elif val_str[0:4] == "-cos":
                trig_str = "ncosq"

            if trig_str is not None:
                vw.writeLine("   assign xform_{rdim}_{cdim} = {trig_str};".format(rdim=dim_list[r], cdim=dim_list[c], trig_str=trig_str))
    ###

    vw.writeLine("")
    vw.writeLine("   // multiplications")
    ###
    vw.writeLine("   // ---")
    for r in range(6):
        for c in range(3):
            val = Xmat[r, c]
            val_str = str(val)
            if "*" in val_str:
                operands = val_str.split("*")
                cst_str, trig_str = operands[0], operands[1] 
                trig_str = trig_str[0:3]
                # converting to 32'd repr
                cst_str = fpga_codegen.to_decimal_string_repr(float(cst_str))

                dim_suffix = dim_list[r] + "_" + dim_list[c]

                vw.writeLine("   cmult#(.WIDTH(WIDTH),.DECIMAL_BITS(DECIMAL_BITS),.C_IN({cst_str}))".format(cst_str=cst_str))
                vw.writeLine("      cmult_xform_{dim_suffix}(.b_in({trig_str}q),.prod_out(xform_{dim_suffix}));".format(dim_suffix=dim_suffix, trig_str=trig_str))
                vw.writeLine("")
    ###

    vw.writeLine("")
    vw.writeLine("   // outputs")
    ###
    xform_out_signals = vw.genXformMatString("xform_out_", xform_bools)
    xform_signals = vw.genXformMatString("xform_", xform_bools)
    vw.setIndentLevel("   ")
    vw.assignVectorLists(xform_out_signals, xform_signals)
    ###

    vw.writeLine("")
    vw.writeLine("endmodule")
