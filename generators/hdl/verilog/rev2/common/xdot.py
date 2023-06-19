import numpy as np
from URDFParser import URDFParser
from FPGACodegen import FPGACodegen
from util import VerilogWriter, FileManager
from rbd_config import dim_list, urdf_file

#---------------------------------------------

parser = URDFParser()
robot = parser.parse(urdf_file)
fpga_codegen = FPGACodegen(robot)

#-------- urdf attrs needed for codegen-ing this file ----------------

num_links = robot.get_num_links_effective()
xform_6x6_bools = fpga_codegen.get_Xmat_sparsity_boolean_matrix_OR()
xform_bools = fpga_codegen.left_half_matrix(xform_6x6_bools)

#-------- File management -------------------

fm = FileManager("xdot.v")
output_file_path = fm.get_output_file_path()
xdot_file = open(output_file_path, "w")

vw = VerilogWriter(xdot_file, dim_list, num_links)

#---------------------------------------------

# Make Transform Matrix String
def makeXform6x6MatString(prefix,dim_list,xform_6x6_bools,skip_str,indent):
   first_row_bool = True
   mat_str = ""
   for row,row_d6 in enumerate(dim_list):
      row_str = ""
      end_of_line_bool = False
      for col,col_d6 in enumerate(dim_list):
         elem_str = ""
         xfm_bool = xform_6x6_bools[row][col]
         if (col == 5):
            end_of_line_bool = True
         elif ((col == 0)&(xform_6x6_bools[row][1] == False)&(xform_6x6_bools[row][2] == False)&\
                          (xform_6x6_bools[row][3] == False)&(xform_6x6_bools[row][4] == False)&(xform_6x6_bools[row][5] == False))|\
              ((col == 1)&(xform_6x6_bools[row][2] == False)&\
                          (xform_6x6_bools[row][3] == False)&(xform_6x6_bools[row][4] == False)&(xform_6x6_bools[row][5] == False))|\
              ((col == 2)&(xform_6x6_bools[row][3] == False)&(xform_6x6_bools[row][4] == False)&(xform_6x6_bools[row][5] == False))|\
              ((col == 3)&(xform_6x6_bools[row][4] == False)&(xform_6x6_bools[row][5] == False))|\
              ((col == 4)&(xform_6x6_bools[row][5] == False)):
            end_of_line_bool = True
         if (end_of_line_bool):
            elem_str = elem_str+prefix+row_d6+"_"+col_d6
            row_str = row_str+elem_str
            break
         else:
            if (xfm_bool):
               elem_str = elem_str+prefix+row_d6+"_"+col_d6+","
            else:
               elem_str = elem_str+skip_str
            row_str = row_str+elem_str
      if (first_row_bool):
         mat_str = mat_str+indent+row_str
      else:
         mat_str = mat_str+","+"\n"+indent+row_str
      first_row_bool = False
   return mat_str

# Make Adder Tree Layer 1 Output Signal Names Per Row, Split by Pairs and Singletons
def makeL1PairsAndSinglesPerRow(dim_list,curr_row):
   row_pair_list = []
   row_sing_list = []
   row_inputs_list = []
   pair_str = ""
   sing_str = ""
   inputs_tuple = []
   pair_count = 0
   for col,col_d6 in enumerate(dim_list):
      if ((curr_row[col])&(pair_count<1)):
         pair_str = pair_str+col_d6
         pair_count = pair_count+1
         inputs_tuple.append(col_d6)
      elif ((curr_row[col])&(pair_count>=1)):
         pair_str = pair_str+col_d6
         row_pair_list.append(pair_str)
         pair_str = ""
         pair_count = 0
         inputs_tuple.append(col_d6)
         row_inputs_list.append(inputs_tuple)
         inputs_tuple = []
      else:
         continue
   if (pair_count>=1):
      row_sing_list.append(pair_str)
   return row_pair_list,row_sing_list,row_inputs_list

# Make Adder Tree Layer 2&3 Output Signal Names Per Row, Split by Pairs and Singletons
def makeL2And3PairsAndSinglesPerRow(dim_list,curr_row_pair_list,curr_row_sing_list):
   all_signals_list = []
   for pair,pair_name in enumerate(curr_row_pair_list):
      all_signals_list.append(pair_name)
   for sing,sing_name in enumerate(curr_row_sing_list):
      all_signals_list.append(sing_name)
   row_pair_list = []
   row_sing_list = []
   row_inputs_list = []
   pair_str = ""
   sing_str = ""
   inputs_tuple = []
   pair_count = 0
   for signal,signal_name in enumerate(all_signals_list):
      if (pair_count<1):
         pair_str = pair_str+signal_name
         pair_count = pair_count+1
         inputs_tuple.append(signal_name)
      elif (pair_count>=1):
         pair_str = pair_str+signal_name
         row_pair_list.append(pair_str)
         pair_str = ""
         pair_count = 0
         inputs_tuple.append(signal_name)
         row_inputs_list.append(inputs_tuple)
         inputs_tuple = []
      else:
         continue
   if (pair_count>=1):
      row_sing_list.append(pair_str)
   return row_pair_list,row_sing_list,row_inputs_list

# Make Adder Tree Output Signals for All Layers, by Pairs and Singletons
def makeAddTreeL1PairsAndSinglesMatrices(dim_list,xform_6x6_bools):
   addtree_l1_pair_mat = []
   addtree_l1_sing_mat = []
   addtree_l1_inputs_mat = []
   for row,curr_row in enumerate(xform_6x6_bools):
      curr_row_l1_pair_list,curr_row_l1_sing_list,curr_row_l1_inputs_list = makeL1PairsAndSinglesPerRow(dim_list,curr_row)
      addtree_l1_pair_mat.append(curr_row_l1_pair_list)
      addtree_l1_sing_mat.append(curr_row_l1_sing_list)
      addtree_l1_inputs_mat.append(curr_row_l1_inputs_list)
   return addtree_l1_pair_mat,addtree_l1_sing_mat,addtree_l1_inputs_mat

# Make Adder Tree Output Signals for All Layers, by Pairs and Singletons
def makeAddTreeL2PairsAndSinglesMatrices(dim_list,xform_6x6_bools,addtree_l1_pair_mat,addtree_l1_sing_mat):
   addtree_l2_pair_mat = []
   addtree_l2_sing_mat = []
   addtree_l2_inputs_mat = []
   for row,curr_row in enumerate(xform_6x6_bools):
      curr_row_l2_pair_list,curr_row_l2_sing_list,curr_row_l2_inputs_list = makeL2And3PairsAndSinglesPerRow(dim_list,addtree_l1_pair_mat[row],addtree_l1_sing_mat[row])
      addtree_l2_pair_mat.append(curr_row_l2_pair_list)
      addtree_l2_sing_mat.append(curr_row_l2_sing_list)
      addtree_l2_inputs_mat.append(curr_row_l2_inputs_list)
   return addtree_l2_pair_mat,addtree_l2_sing_mat,addtree_l2_inputs_mat

# Make Adder Tree Output Signals for All Layers, by Pairs and Singletons
def makeAddTreeL3PairsAndSinglesMatrices(dim_list,xform_6x6_bools,addtree_l2_pair_mat,addtree_l2_sing_mat):
   addtree_l3_pair_mat = []
   addtree_l3_sing_mat = []
   addtree_l3_inputs_mat = []
   for row,curr_row in enumerate(xform_6x6_bools):
      curr_row_l3_pair_list,curr_row_l3_sing_list,curr_row_l3_inputs_list = makeL2And3PairsAndSinglesPerRow(dim_list,addtree_l2_pair_mat[row],addtree_l2_sing_mat[row])
      addtree_l3_pair_mat.append(curr_row_l3_pair_list)
      addtree_l3_sing_mat.append(curr_row_l3_sing_list)
      addtree_l3_inputs_mat.append(curr_row_l3_inputs_list)
   return addtree_l3_pair_mat,addtree_l3_sing_mat,addtree_l3_inputs_mat

#-------------------------------------------------------------------------------

# Make Transform Input Port
def makeXform6x6InputPort(prefix,dim_list,xform_6x6_bools,skip_str,indent):
   mat_str = ""
   mat_str = mat_str+"   input  signed[(WIDTH-1):0]"+"\n"
   xfm_str = ""
   xfm_str = makeXform6x6MatString(prefix,dim_list,xform_6x6_bools,skip_str,indent)
   mat_str = mat_str+xfm_str
   mat_str = mat_str+","+"\n"
   return mat_str

# Make Transform Wires
def makeXform6x6Wires(prefix,dim_list,xform_6x6_bools,skip_str,indent):
   mat_str = ""
   mat_str = mat_str+"   wire signed[(WIDTH-1):0]"+"\n"
   xfm_str = ""
   xfm_str = makeXform6x6MatString(prefix,dim_list,xform_6x6_bools,skip_str,indent)
   mat_str = mat_str+xfm_str
   mat_str = mat_str+";"+"\n"
   return mat_str

# Make Add Tree Layer Wires
def makeAddTreeLayerWires(prefix,dim_list,indent,addtree_layer_pair_mat):
   first_row_bool = True
   last_row_bool = False
   mat_str = ""
   mat_str = mat_str+"   wire signed[(WIDTH-1):0]"
   for row,row_d6 in enumerate(dim_list):
      last_row_bool = (row == (len(dim_list)-1))
      row_pair_list = addtree_layer_pair_mat[row]
      if (row_pair_list):
         if (first_row_bool):
            mat_str = mat_str+"\n"+indent
         else:
            mat_str = mat_str+","+"\n"+indent
         first_pair_bool = True
         for pair_str in row_pair_list:
            row_str = ""
            if (first_pair_bool):
               row_str = row_str
            else:
               row_str = row_str+","
            row_str = row_str+prefix+row_d6+"_"+pair_str
            mat_str = mat_str+row_str
            first_pair_bool = False
         first_row_bool = False
   mat_str = mat_str+";"+"\n"
   return mat_str

# Make Multiplier Units String
def makeMultUnitsString(dim_list,xform_6x6_bools,indent):
   a_prefix = "xform_in_"
   b_prefix = "vec_in_"
   prod_prefix = "xvec_"
   mult_prefix = "mult_xvec_"
   mat_str = ""
   for row,row_d6 in enumerate(dim_list):
      row_str = ""
      for col,col_d6 in enumerate(dim_list):
         elem_str = ""
         xfm_bool = xform_6x6_bools[row][col]
         if (xfm_bool):
            elem_str = elem_str+indent+"mult#(.WIDTH(WIDTH),.DECIMAL_BITS(DECIMAL_BITS))"+"\n"
            elem_str = elem_str+indent+indent+mult_prefix+row_d6+"_"+col_d6+"(.a_in("+a_prefix+\
               row_d6+"_"+col_d6+"),.b_in("+b_prefix+col_d6+"),.prod_out("+prod_prefix+row_d6+"_"+col_d6+"));"+"\n"
         row_str = row_str+elem_str
      mat_str = mat_str+row_str+"\n"
   return mat_str

# Make Adder Units Layer String
def makeAddUnitsLayerString(dim_list,xform_6x6_bools,indent,add_l1_pair_mat,add_l1_inputs_mat):
   a_prefix = "xvec_"
   b_prefix = "xvec_"
   sum_prefix = "xvec_"
   add_prefix = "add_xvec_"
   mat_str = ""
   for row,row_d6 in enumerate(dim_list):
      row_str = ""
      for pair,pair_str in enumerate(add_l1_pair_mat[row]):
         inputs_tuple = add_l1_inputs_mat[row][pair]
         inputs_a = inputs_tuple[0]
         inputs_b = inputs_tuple[1]
         elem_str = ""
         elem_str = elem_str+indent+"add#(.WIDTH(WIDTH),.DECIMAL_BITS(DECIMAL_BITS))"+"\n"
         elem_str = elem_str+indent+indent+add_prefix+row_d6+"_"+pair_str+"(.a_in("+a_prefix+\
            row_d6+"_"+inputs_a+"),.b_in("+b_prefix+row_d6+"_"+inputs_b+"),.sum_out("+sum_prefix+row_d6+"_"+pair_str+"));"+"\n"
         row_str = row_str+elem_str
      if (add_l1_pair_mat[row]):
         mat_str = mat_str+row_str+"\n"
   return mat_str

# Make Mcross Mux String
def makeMcrossMuxString(dim_list,indent):
   mat_str = ""
   out_vec = []
   out_prefix = "xvec_"
   mcross_check = " = mcross ? "
   for row,row_d6 in enumerate(dim_list):
      out_pair = addtree_l3_pair_mat[row]
      out_sing = addtree_l3_sing_mat[row]
      vec_str = ""
      if (out_pair):
         vec_str = vec_str+out_prefix+row_d6+"_"+out_pair[0]
      else:
         vec_str = vec_str+out_prefix+row_d6+"_"+out_sing[0]
      out_vec.append(vec_str)
   mat_str = mat_str+indent+"assign "+out_prefix+"out_"+dim_list[0]+mcross_check+" "+out_vec[1]+indent+indent+" : "+out_vec[0]+";"+"\n"
   mat_str = mat_str+indent+"assign "+out_prefix+"out_"+dim_list[1]+mcross_check+"-"+out_vec[0]+indent+indent+" : "+out_vec[1]+";"+"\n"
   mat_str = mat_str+indent+"assign "+out_prefix+"out_"+dim_list[2]+mcross_check+" "+"0 "+indent+indent+indent+indent+indent+indent+" : "+out_vec[2]+";"+"\n"
   mat_str = mat_str+indent+"assign "+out_prefix+"out_"+dim_list[3]+mcross_check+" "+out_vec[4]+" : "+out_vec[3]+";"+"\n"
   mat_str = mat_str+indent+"assign "+out_prefix+"out_"+dim_list[4]+mcross_check+"-"+out_vec[3]+" : "+out_vec[4]+";"+"\n"
   mat_str = mat_str+indent+"assign "+out_prefix+"out_"+dim_list[5]+mcross_check+" "+"0 "+indent+indent+indent+indent+indent+indent+" : "+out_vec[5]+";"+"\n"
   return mat_str 

#-------------------------------------------------------------------------------
xdot_file.write("`timescale 1ns / 1ps"+"\n")
xdot_file.write("\n")
xdot_file.write("// Transformation Matrix Multiplication by a Vector"+"\n")
xdot_file.write("//"+"\n")
# TODO: Comments won't match when X matrix changes. Not important, but could fix later.
xdot_file.write("// (23 mult, 17 add)"+"\n")
xdot_file.write("//       0  1  2  3  4  5 vec"+"\n")
xdot_file.write("//      AX AY AZ LX LY LZ"+"\n")
xdot_file.write("// 0 AX  0  1  2"+"\n")
xdot_file.write("// 1 AY  3  4  5"+"\n")
xdot_file.write("// 2 AZ     6  7"+"\n")
xdot_file.write("// 3 LX  8  9 10 11 12 13"+"\n")
xdot_file.write("// 4 LY 14 15 16 17 18 19"+"\n")
xdot_file.write("// 5 LZ 20          21 22"+"\n")
xdot_file.write("// xvec                  xform"+"\n")
xdot_file.write("\n")
xdot_file.write("//------------------------------------------------------------------------------"+"\n")
xdot_file.write("// xdot Module"+"\n")
xdot_file.write("//------------------------------------------------------------------------------"+"\n")
xdot_file.write("module xdot#(parameter WIDTH = 32,parameter DECIMAL_BITS = 16)("+"\n")
xdot_file.write("   // mcross boolean"+"\n")
xdot_file.write("   input  mcross,"+"\n")
#-------------------------------------------------------------------------------
xdot_file.write("   // xform_in, 23 values"+"\n")
mat_str = makeXform6x6InputPort("xform_in_",dim_list,xform_6x6_bools,"               ","      ")
xdot_file.write(mat_str)
#-------------------------------------------------------------------------------
xdot_file.write("   // vec_in, 6 values"+"\n")
xdot_file.write("   input  signed[(WIDTH-1):0]"+"\n")
xdot_file.write("      vec_in_AX,vec_in_AY,vec_in_AZ,vec_in_LX,vec_in_LY,vec_in_LZ,"+"\n")
xdot_file.write("   // xvec_out, 6 values"+"\n")
xdot_file.write("   output signed[(WIDTH-1):0]"+"\n")
xdot_file.write("      xvec_out_AX,xvec_out_AY,xvec_out_AZ,xvec_out_LX,xvec_out_LY,xvec_out_LZ"+"\n")
xdot_file.write("   );"+"\n")
xdot_file.write("\n")
xdot_file.write("   // internal wires and state"+"\n")
xdot_file.write("   // results from the multiplications"+"\n")
#-------------------------------------------------------------------------------
mat_str = makeXform6x6Wires("xvec_",dim_list,xform_6x6_bools,"           ","      ")
xdot_file.write(mat_str)
#-------------------------------------------------------------------------------
addtree_l1_pair_mat = []
addtree_l1_sing_mat = []
addtree_l1_inputs_mat = []
addtree_l2_pair_mat = []
addtree_l2_sing_mat = []
addtree_l2_inputs_mat = []
addtree_l3_pair_mat = []
addtree_l3_sing_mat = []
addtree_l3_inputs_mat = []
addtree_l1_pair_mat,addtree_l1_sing_mat,addtree_l1_inputs_mat = makeAddTreeL1PairsAndSinglesMatrices(dim_list,xform_6x6_bools)
addtree_l2_pair_mat,addtree_l2_sing_mat,addtree_l2_inputs_mat = makeAddTreeL2PairsAndSinglesMatrices(dim_list,xform_6x6_bools,addtree_l1_pair_mat,addtree_l1_sing_mat)
addtree_l3_pair_mat,addtree_l3_sing_mat,addtree_l3_inputs_mat = makeAddTreeL3PairsAndSinglesMatrices(dim_list,xform_6x6_bools,addtree_l2_pair_mat,addtree_l2_sing_mat)
#-------------------------------------------------------------------------------
xdot_file.write("   // results from layer 1 of additions"+"\n")
#-------------------------------------------------------------------------------
mat_str = makeAddTreeLayerWires("xvec_",dim_list,"      ",addtree_l1_pair_mat)
xdot_file.write(mat_str)
#-------------------------------------------------------------------------------
xdot_file.write("   // results from layer 2 of additions"+"\n")
#-------------------------------------------------------------------------------
mat_str = makeAddTreeLayerWires("xvec_",dim_list,"      ",addtree_l2_pair_mat)
xdot_file.write(mat_str)
#-------------------------------------------------------------------------------
xdot_file.write("   // results from layer 3 of additions"+"\n")
#-------------------------------------------------------------------------------
mat_str = makeAddTreeLayerWires("xvec_",dim_list,"      ",addtree_l3_pair_mat)
xdot_file.write(mat_str)
xdot_file.write("\n")
#-------------------------------------------------------------------------------
xdot_file.write("   // multiplications (23 in ||)"+"\n")
#-------------------------------------------------------------------------------
mat_str = makeMultUnitsString(dim_list,xform_6x6_bools,"   ")
xdot_file.write(mat_str)
#-------------------------------------------------------------------------------
xdot_file.write("   // layer 1 of additions (10 in ||)"+"\n")
#-------------------------------------------------------------------------------
mat_str = makeAddUnitsLayerString(dim_list,xform_6x6_bools,"   ",addtree_l1_pair_mat,addtree_l1_inputs_mat)
xdot_file.write(mat_str)
#-------------------------------------------------------------------------------
xdot_file.write("   // layer 2 of additions (5 in ||)"+"\n")
#-------------------------------------------------------------------------------
mat_str = makeAddUnitsLayerString(dim_list,xform_6x6_bools,"   ",addtree_l2_pair_mat,addtree_l2_inputs_mat)
xdot_file.write(mat_str)
#-------------------------------------------------------------------------------
xdot_file.write("   // layer 3 of additions (2 in ||)"+"\n")
#-------------------------------------------------------------------------------
mat_str = makeAddUnitsLayerString(dim_list,xform_6x6_bools,"   ",addtree_l3_pair_mat,addtree_l3_inputs_mat)
xdot_file.write(mat_str)
#-------------------------------------------------------------------------------
xdot_file.write("   // muxes for mcross"+"\n")
#-------------------------------------------------------------------------------
mat_str = makeMcrossMuxString(dim_list,"   ")
xdot_file.write(mat_str)
xdot_file.write("\n")
#-------------------------------------------------------------------------------
xdot_file.write("endmodule"+"\n")
#-------------------------------------------------------------------------------

# Close file
xdot_file.close()
