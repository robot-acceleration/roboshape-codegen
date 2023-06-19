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

imatrix_list = fpga_codegen.construct_inertia_decimal_repr()
# I Matrix Sparsity Boolean List
imat_6x6_bools = fpga_codegen.get_Imat_sparsity_boolean_matrix_OR()

#-------- File management -------------------

fm = FileManager("idots{num_links}.v".format(num_links=num_links))
output_file_path = fm.get_output_file_path()
idots_file = open(output_file_path, "w")

vw = VerilogWriter(idots_file, dim_list, num_links)

#------------ Bitwidth stuff -----------------

# Find Bitwidth of # Links
bitwidth_num_links = vw.get_bitwidth(num_links)
bitwidth_num_links_str = str(bitwidth_num_links)

#---------------------------------------------

# List of I Matrices for All Links
#imatrix_list  = [[["32'd10564", "-32'd0", "-32'd0", "", "-32'd31457", "-32'd7864"],
#                  ["-32'd0", "32'd9673", "32'd944", "32'd31457", "", "-32'd0"],
#                  ["-32'd0", "32'd944", "32'd1547", "32'd7864", "32'd0", ""],
#                  ["", "32'd31457", "32'd7864", "32'd262144", "", ""],
#                  ["-32'd31457", "", "32'd0", "", "32'd262144", ""],
#                  ["-32'd7864", "-32'd0", "", "", "", "32'd262144"]],
#                 [["32'd4652", "-32'd5", "-32'd3", "", "-32'd11010", "32'd15466"],
#                  ["-32'd5", "32'd1642", "-32'd650", "32'd11010", "", "-32'd79"],
#                  ["-32'd3", "-32'd650", "32'd3796", "-32'd15466", "32'd79", ""],
#                  ["", "32'd11010", "-32'd15466", "32'd262144", "", ""],
#                  ["-32'd11010", "", "32'd79", "", "32'd262144", ""],
#                  ["32'd15466", "-32'd79", "", "", "", "32'd262144"]],
#                 [["32'd8743", "-32'd0", "-32'd0", "", "-32'd25559", "32'd5898"],
#                  ["-32'd0", "32'd8238", "-32'd767", "32'd25559", "", "-32'd0"],
#                  ["-32'd0", "-32'd767", "32'd832", "-32'd5898", "32'd0", ""],
#                  ["", "32'd25559", "-32'd5898", "32'd196608", "", ""],
#                  ["-32'd25559", "", "32'd0", "", "32'd196608", ""],
#                  ["32'd5898", "-32'd0", "", "", "", "32'd196608"]],
#                 [["32'd2965", "-32'd0", "-32'd0", "", "-32'd6016", "32'd11855"],
#                  ["-32'd0", "32'd860", "-32'd403", "32'd6016", "", "-32'd0"],
#                  ["-32'd0", "-32'd403", "32'd2695", "-32'd11855", "32'd0", ""],
#                  ["", "32'd6016", "-32'd11855", "32'd176947", "", ""],
#                  ["-32'd6016", "", "32'd0", "", "32'd176947", ""],
#                  ["32'd11855", "-32'd0", "", "", "", "32'd176947"]],
#                 [["32'd2003", "-32'd0", "-32'd1", "", "-32'd8467", "32'd2340"],
#                  ["-32'd0", "32'd1823", "-32'd178", "32'd8467", "", "-32'd11"],
#                  ["-32'd1", "-32'd178", "32'd377", "-32'd2340", "32'd11", ""],
#                  ["", "32'd8467", "-32'd2340", "32'd111411", "", ""],
#                  ["-32'd8467", "", "32'd11", "", "32'd111411", ""],
#                  ["32'd2340", "-32'd11", "", "", "", "32'd111411"]],
#                 [["32'd328", "-32'd0", "-32'd0", "", "-32'd47", "32'd71"],
#                  ["-32'd0", "32'd236", "-32'd0", "32'd47", "", "-32'd0"],
#                  ["-32'd0", "-32'd0", "32'd308", "-32'd71", "32'd0", ""],
#                  ["", "32'd47", "-32'd71", "32'd117965", "", ""],
#                  ["-32'd47", "", "32'd0", "", "32'd117965", ""],
#                  ["32'd71", "-32'd0", "", "", "", "32'd117965"]],
#                 [["32'd73", "-32'd0", "-32'd0", "", "-32'd393", "32'd0"],
#                  ["-32'd0", "32'd73", "-32'd0", "32'd393", "", "-32'd0"],
#                  ["-32'd0", "-32'd0", "32'd66", "-32'd0", "32'd0", ""],
#                  ["", "32'd393", "-32'd0", "32'd19661", "", ""],
#                  ["-32'd393", "", "32'd0", "", "32'd19661", ""],
#                  ["32'd0", "-32'd0", "", "", "", "32'd19661"]]]

#-------------------------------------------------------------------------------

# Make I Matrix Parameters String
def makeIParametersMatString(dim_list,imat_6x6_bools,prefix,imatrix,skip_str,indent):
   first_row_bool = True
   mat_str = ""
   for row,row_d6 in enumerate(dim_list):
      row_str = ""
      end_of_line_bool = False
      for col,col_d6 in enumerate(dim_list):
         elem_str = ""
         imat_bool = imat_6x6_bools[row][col]
         if (col == 5):
            end_of_line_bool = True
         elif ((col == 0)&(imat_6x6_bools[row][1] == False)&(imat_6x6_bools[row][2] == False)&\
                          (imat_6x6_bools[row][3] == False)&(imat_6x6_bools[row][4] == False)&(imat_6x6_bools[row][5] == False))|\
              ((col == 1)&(imat_6x6_bools[row][2] == False)&\
                          (imat_6x6_bools[row][3] == False)&(imat_6x6_bools[row][4] == False)&(imat_6x6_bools[row][5] == False))|\
              ((col == 2)&(imat_6x6_bools[row][3] == False)&(imat_6x6_bools[row][4] == False)&(imat_6x6_bools[row][5] == False))|\
              ((col == 3)&(imat_6x6_bools[row][4] == False)&(imat_6x6_bools[row][5] == False))|\
              ((col == 4)&(imat_6x6_bools[row][5] == False)):
            end_of_line_bool = True
         if (end_of_line_bool):
            elem_str = elem_str+prefix+row_d6+"_"+col_d6+" = "+imatrix[row][col]
            row_str = row_str+elem_str
            break
         else:
            if (imat_bool):
               elem_str = elem_str+prefix+row_d6+"_"+col_d6+" = "+imatrix[row][col]+","
            else:
               elem_str = elem_str+skip_str
            row_str = row_str+elem_str
      if (first_row_bool):
         mat_str = mat_str+indent+row_str
      else:
         mat_str = mat_str+","+"\n"+indent+row_str
      first_row_bool = False
   return mat_str

# Make I Matrix Port Assignment String
def makeIPortAssignmentMatString(dim_list,imat_6x6_bools,prefix,prefix_in,skip_str,indent):
   first_row_bool = True
   mat_str = ""
   for row,row_d6 in enumerate(dim_list):
      row_str = ""
      end_of_line_bool = False
      for col,col_d6 in enumerate(dim_list):
         elem_str = ""
         imat_bool = imat_6x6_bools[row][col]
         if (col == 5):
            end_of_line_bool = True
         elif ((col == 0)&(imat_6x6_bools[row][1] == False)&(imat_6x6_bools[row][2] == False)&\
                          (imat_6x6_bools[row][3] == False)&(imat_6x6_bools[row][4] == False)&(imat_6x6_bools[row][5] == False))|\
              ((col == 1)&(imat_6x6_bools[row][2] == False)&\
                          (imat_6x6_bools[row][3] == False)&(imat_6x6_bools[row][4] == False)&(imat_6x6_bools[row][5] == False))|\
              ((col == 2)&(imat_6x6_bools[row][3] == False)&(imat_6x6_bools[row][4] == False)&(imat_6x6_bools[row][5] == False))|\
              ((col == 3)&(imat_6x6_bools[row][4] == False)&(imat_6x6_bools[row][5] == False))|\
              ((col == 4)&(imat_6x6_bools[row][5] == False)):
            end_of_line_bool = True
         if (end_of_line_bool):
            elem_str = elem_str+"."+prefix+row_d6+"_"+col_d6+"("+prefix_in+row_d6+"_"+col_d6+")"
            row_str = row_str+elem_str
            break
         else:
            if (imat_bool):
               elem_str = elem_str+"."+prefix+row_d6+"_"+col_d6+"("+prefix_in+row_d6+"_"+col_d6+")"+","
            else:
               elem_str = elem_str+skip_str
            row_str = row_str+elem_str
      if (first_row_bool):
         mat_str = mat_str+indent+row_str
      else:
         mat_str = mat_str+","+"\n"+indent+row_str
      first_row_bool = False
   return mat_str

#-------------------------------------------------------------------------------

# Make I Matrix Parameters String Per Link
def makeIParametersMatStringPerLink(dim_list,imat_6x6_bools,link_str,prefix,imatrix,skip_str,indent):
   mat_str = ""
   if (link_str == "1"):
      mat_str = mat_str+"   // parameters"+"\n"
   else:
      mat_str = mat_str+"   // -----------------------------------------------------------------------"+"\n"
   mat_str = mat_str+"   // Link "+link_str+"\n"
   mat_str = mat_str+"   parameter signed"+"\n"
   param_str = ""
   param_str = makeIParametersMatString(dim_list,imat_6x6_bools,prefix,imatrix,skip_str,indent)
   mat_str = mat_str+param_str+";"+"\n"
   return mat_str

# Make I Matrix Port Assignment String Per Link
def makeIPortAssignmentMatStringPerLink(dim_list,imat_6x6_bools,link_str,prefix,prefix_in,skip_str,indent):
   mat_str = ""
   mat_str = mat_str+"   // idot, Link "+link_str+"\n"
   mat_str = mat_str+"   idot#(.WIDTH(WIDTH),.DECIMAL_BITS(DECIMAL_BITS),"+"\n"
   mat_str = mat_str+"      // IMAT_IN, 24 values"+"\n"
   param_str = ""
   param_str = makeIPortAssignmentMatString(dim_list,imat_6x6_bools,prefix,prefix_in,skip_str,indent)
   mat_str = mat_str+param_str+")"+"\n"
   mat_str = mat_str+"      idot"+link_str+"("+"\n"
   mat_str = mat_str+"      // vec_in, 6 values"+"\n"
   mat_str = mat_str+"      .vec_in_AX(vec_in_AX),.vec_in_AY(vec_in_AY),.vec_in_AZ(vec_in_AZ),.vec_in_LX(vec_in_LX),.vec_in_LY(vec_in_LY),.vec_in_LZ(vec_in_LZ),"+"\n"
   mat_str = mat_str+"      // ivec_out, 6 values"+"\n"
   mat_str = mat_str+"      .ivec_out_AX(ivec_l"+link_str+"_out_AX),.ivec_out_AY(ivec_l"+link_str+"_out_AY),.ivec_out_AZ(ivec_l"+link_str+"_out_AZ),.ivec_out_LX(ivec_l"+link_str+"_out_LX),.ivec_out_LY(ivec_l"+link_str+"_out_LY),.ivec_out_LZ(ivec_l"+link_str+"_out_LZ)"+"\n"
   mat_str = mat_str+"      );"+"\n"
   return mat_str

# Make I Vector Results Wire String Per Link
def makeIVectorResultsWireStringPerLink(link_str):
   mat_str = ""
   mat_str = mat_str+"   // results, Link "+link_str+""+"\n"
   mat_str = mat_str+"   wire signed[(WIDTH-1):0]"+"\n"
   mat_str = mat_str+"      ivec_l"+link_str+"_out_AX,ivec_l"+link_str+"_out_AY,ivec_l"+link_str+"_out_AZ,ivec_l"+link_str+"_out_LX,ivec_l"+link_str+"_out_LY,ivec_l"+link_str+"_out_LZ;"+"\n"
   return mat_str

# Make I Vector Output Muxes Per 6D Dimension
def makeIVectorOutputMuxesPer6D(num_links,d6_str):
   mat_str = ""
   mat_str = mat_str+"   // "+d6_str+"\n"
   for curr_link in range(num_links): 
      link_str = str(curr_link+1)
      if (link_str == "1"):
         mat_str = mat_str+"   assign ivec_out_"+d6_str+" = "
      else:
         mat_str = mat_str+"                        "
      mat_str = mat_str+"(link_in == "+bitwidth_num_links_str+"'d"+link_str+") ?  ivec_l"+link_str+"_out_"+d6_str+" :"
      if (link_str == str(num_links)):
         mat_str = mat_str+" 32'd0;"+"\n"
      else:
         mat_str = mat_str+"\n"
   return mat_str

#-------------------------------------------------------------------------------
idots_file.write("`timescale 1ns / 1ps"+"\n")
idots_file.write("\n")
idots_file.write("// Inertia Matrix Multiplication by a Vector"+"\n")
idots_file.write("\n")
idots_file.write("//------------------------------------------------------------------------------"+"\n")
idots_file.write("// idots{num_links} Module".format(num_links=num_links)+"\n")
idots_file.write("//------------------------------------------------------------------------------"+"\n")
idots_file.write("module idots{num_links}#(parameter WIDTH = 32,parameter DECIMAL_BITS = 16)(".format(num_links=num_links)+"\n")
idots_file.write("   // link_in"+"\n")
idots_file.write("   input  ["+str(vw.get_bitwidth_array_def(num_links))+":0]"+"\n")
idots_file.write("      link_in,"+"\n")
idots_file.write("   // vec_in, 6 values"+"\n")
idots_file.write("   input  signed[(WIDTH-1):0]"+"\n")
idots_file.write("      vec_in_AX,vec_in_AY,vec_in_AZ,vec_in_LX,vec_in_LY,vec_in_LZ,"+"\n")
idots_file.write("   // ivec_out, 6 values"+"\n")
idots_file.write("   output signed[(WIDTH-1):0]"+"\n")
idots_file.write("      ivec_out_AX,ivec_out_AY,ivec_out_AZ,ivec_out_LX,ivec_out_LY,ivec_out_LZ"+"\n")
idots_file.write("   );"+"\n")
idots_file.write("\n")
#-------------------------------------------------------------------------------
link_str = ""
for curr_link in range(num_links):
   link_str = str(curr_link+1)
   prefix = "IMAT_L"+link_str+"_IN_"
   mat_str = makeIParametersMatStringPerLink(dim_list,imat_6x6_bools,link_str,prefix,imatrix_list[curr_link],"                                  ","      ")
   idots_file.write(mat_str)
idots_file.write("\n")
#-------------------------------------------------------------------------------
idots_file.write("   // internal wires and state"+"\n")
#-------------------------------------------------------------------------------
link_str = ""
for curr_link in range(num_links):
   link_str = str(curr_link+1)
   mat_str = makeIVectorResultsWireStringPerLink(link_str)
   idots_file.write(mat_str)
idots_file.write("\n")
#-------------------------------------------------------------------------------
idots_file.write("   // idots"+"\n")
#-------------------------------------------------------------------------------
link_str = ""
for curr_link in range(num_links):
   link_str = str(curr_link+1)
   prefix = "IMAT_IN_"
   prefix_in = "IMAT_L"+link_str+"_IN_"
   mat_str = makeIPortAssignmentMatStringPerLink(dim_list,imat_6x6_bools,link_str,prefix,prefix_in,"                              ","      ")
   idots_file.write(mat_str)
idots_file.write("\n")
#-------------------------------------------------------------------------------
idots_file.write("   // output muxes"+"\n")
#-------------------------------------------------------------------------------
for d6_str in dim_list:
   mat_str = makeIVectorOutputMuxesPer6D(num_links,d6_str)
   idots_file.write(mat_str)
#-------------------------------------------------------------------------------
idots_file.write("\n")
#-------------------------------------------------------------------------------
idots_file.write("endmodule"+"\n")
#-------------------------------------------------------------------------------

# Close file
idots_file.close()
