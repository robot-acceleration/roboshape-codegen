from .VectorGenerator import VectorGenerator
from .BaseWriter import BaseWriter
import numpy as np

class VerilogWriter(BaseWriter):
    def __init__(self, file, dim_list, num_links, block_size=-1):
        super(VerilogWriter, self).__init__(file, dim_list, num_links, block_size)

    def assignVectorLists(self, list1, list2):
        associated_vectors = self.associateVectorLists(list1, list2)
        for lvalue, rvalue in associated_vectors:
            self.file.write(self.indent + "assign {lvalue} = {rvalue};\n".format(lvalue=lvalue, rvalue=rvalue))

    def regUpdateVectorLists(self, list1, list2):
        associated_vectors = self.associateVectorLists(list1, list2)
        for lvalue, rvalue in associated_vectors:
            self.file.write(self.indent + "{lvalue} <= {rvalue};\n".format(lvalue=lvalue, rvalue=rvalue))

    def assignMux(self, lhs, rhs_if, rhs_true, rhs_false):
        self.writeLine(self.indent + "assign {lhs} = ({rhs_if}) ? {rhs_true} : {rhs_false};".format(lhs=lhs, rhs_if=rhs_if, rhs_true=rhs_true, rhs_false=rhs_false))

    def portAssignmentVectorLists(self, ports, signals, lineSepElements=-1, last=False):
        associated_vectors = self.associateVectorLists(ports, signals)
        port_signal_connection_list = []
        for lvalue, rvalue in associated_vectors:
            port_signal_connection_list.append(".{lvalue}({rvalue})".format(lvalue=lvalue, rvalue=rvalue))

        self.writeCommaSep(port_signal_connection_list, lineSepElements, last)

    def writeMultiplierVectorLists(self, a, b, prod, unit_name):
        associated_vectors = self.associateVectorLists(a, b, prod, unit_name)
        for a_in, b_in, prod_out, u in associated_vectors:
            self.file.write(self.indent + "mult#(.WIDTH(WIDTH), .DECIMAL_BITS(DECIMAL_BITS)) \
                    {u}(.a_in({a_in}),.b_in({b_in}),.prod_out({prod_out}));\n".format( \
                    a_in=a_in, b_in=b_in, prod_out=prod_out, u=u))

    def writeMultiplierDictList(self, mult_units):
        for mult in mult_units:
            a_in = mult["a_in"]
            b_in = mult["b_in"]
            prod_out = mult["prod_out"]
            u = mult["unit_name"]
            self.file.write(self.indent + "mult#(.WIDTH(WIDTH), .DECIMAL_BITS(DECIMAL_BITS))\n      {u}(.a_in({a_in}),.b_in({b_in}),.prod_out({prod_out}));\n".format(a_in=a_in, b_in=b_in, prod_out=prod_out, u=u))

    def writeAdderDictList(self, add_units):
        for add in add_units:
            a_in = add["a_in"]
            b_in = add["b_in"]
            sum_out = add["sum_out"]
            u = add["unit_name"]
            self.file.write(self.indent + "add#(.WIDTH(WIDTH), .DECIMAL_BITS(DECIMAL_BITS))\n      {u}(.a_in({a_in}),.b_in({b_in}),.sum_out({sum_out}));\n".format(a_in=a_in, b_in=b_in, sum_out=sum_out, u=u))
    
    ### Xform stuff

    def getCommaSepXformSparseString(self, sparse_string_list, indent, lineSepElements=3, end=""):
        assert end == "" or end == ";" or end == ",", "have to end list with legal eol marker"
        # set skip_str to len of first non-null string
        for s in sparse_string_list:
            if s:
                skip_str = " " * len(s)
                break

        mat_str = ""
        # print lineSepElements per line
        chunked_list = [sparse_string_list[i:i + lineSepElements] for i in range(0, len(sparse_string_list), lineSepElements)]

        i = 0
        for c, chunk in enumerate(chunked_list):
            mat_str += indent
            for s in chunk:
                if s:
                    # are there non-None elements left after current entry?
                    more_elements = False
                    for remaining in sparse_string_list[i+1:]:
                        if remaining:
                            more_elements = True
                            break
                    mat_str += s + ","*more_elements
                else:
                    mat_str += skip_str
                i += 1
            if c < len(chunked_list) - 1:
                mat_str += "\n"
            else:
                mat_str += end
        return mat_str

    # Make Transform Matrix String
    def makeXformMatString(self,prefix,xform_bools,skip_str,indent):
        sparse_string_list = []

        for r, dim_r in enumerate(self.dim_list):
            for c, dim_c in enumerate(self.dim_list[:3]):
                if xform_bools[r][c]:
                    signal = "{prefix}{dim_r}_{dim_c}".format(prefix=prefix, dim_r=dim_r, dim_c=dim_c)
                    sparse_string_list.append(signal)
                else:
                    sparse_string_list.append(None)

        return self.getCommaSepXformSparseString(sparse_string_list, indent)

    # Make Transform Matrix Port Assignment String
    def makeXformMatPortAssignmentString(self,prefix,prefix_in,xform_bools,skip_str,indent):
        sparse_string_list = []

        for r, dim_r in enumerate(self.dim_list):
            for c, dim_c in enumerate(self.dim_list[:3]):
                if xform_bools[r][c]:
                    port = "{prefix}{dim_r}_{dim_c}".format(prefix=prefix, dim_r=dim_r, dim_c=dim_c)
                    signal = "{prefix_in}{dim_r}_{dim_c}".format(prefix_in=prefix_in, dim_r=dim_r, dim_c=dim_c)
                    connection = ".{port}({signal})".format(port=port, signal=signal)
                    sparse_string_list.append(connection)
                else:
                    sparse_string_list.append(None)

        return self.getCommaSepXformSparseString(sparse_string_list, indent)

    # Make Transform Matrix 1-Conditional Assignment String
    def makeXformMat1CondAssignString(self,prefix,cond_str,prefix_true,prefix_false,xform_bools,indent):
       first_row_bool = True
       mat_str = ""
       for row,row_d6 in enumerate(self.dim_list):
          for col in range(0,3):
             col_d6 = self.dim_list[col]
             xfm_bool = xform_bools[row][col]
             asn_str = ""
             if (xfm_bool):
                asn_str = asn_str+"assign "+prefix+row_d6+"_"+col_d6+"  = "+cond_str+" ? "+prefix_true+row_d6+"_"+col_d6+" : "+prefix_false+row_d6+"_"+col_d6+";"
                if (first_row_bool):
                   mat_str = mat_str+indent+asn_str
                else:
                   mat_str = mat_str+"\n"+indent+asn_str
             first_row_bool = False
       return mat_str

    # Make 6x6 Transform Matrix Port Assignment String
    def make6x6XformMatPortAssignmentString(self,prefix,prefix_in,xform_bools,skip_str,indent):
        sparse_string_list = []
        dim_list = self.dim_list

        for r, dim_r in enumerate(dim_list):
            for c, dim_c in enumerate(dim_list):
                # port name remains unchanged
                port = "{prefix}{dim_r}_{dim_c}".format(prefix=prefix, dim_r=dim_r, dim_c=dim_c)

                #   E   0
                # -Erx  E
                r_mod3 = r % 3
                c_mod3 = c % 3
                signal = "{prefix_in}{dim_r}_{dim_c}".format(prefix_in=prefix_in, dim_r=dim_r, dim_c=dim_c)

                xform_6x6_bool = None
                if c < 3:
                    xform_6x6_bool = xform_bools[r][c]
                elif r < 3 and c >= 3:
                    xform_6x6_bool = False
                else:
                    xform_6x6_bool = xform_bools[r_mod3][c_mod3]
                    signal = "{prefix_in}{dim_r}_{dim_c}".format(prefix_in=prefix_in, dim_r=dim_list[r_mod3], dim_c=dim_list[c_mod3])

                if xform_6x6_bool:
                    connection = ".{port}({signal})".format(port=port, signal=signal)
                    sparse_string_list.append(connection)
                else:
                    sparse_string_list.append(None)

        return self.getCommaSepXformSparseString(sparse_string_list, indent, lineSepElements=6)

    # Make Transform Matrix Assignment String
    def makeXformMatAssignmentString(self,prefix,prefix_in,xform_bools,indent):
       first_row_bool = True
       mat_str = ""
       for row,row_d6 in enumerate(self.dim_list):
          for col in range(0,3):
             col_d6 = self.dim_list[col]
             xfm_bool = xform_bools[row][col]
             assignment = ""
             assignment = assignment+prefix_in+row_d6+"_"+col_d6
             asn_str = ""
             if (xfm_bool):
                asn_str = asn_str+"assign "+prefix+row_d6+"_"+col_d6+" = "+assignment+";"
                if (first_row_bool):
                   mat_str = mat_str+indent+asn_str
                else:
                   mat_str = mat_str+"\n"+indent+asn_str
                first_row_bool = False
             else:
                mat_str = mat_str
       return mat_str

    # Make Transform Matrix Nonblocking Assignment String
    def makeXformMatNonblockingAssignmentString(self,prefix,prefix_in,xform_bools,indent):
       first_row_bool = True
       mat_str = ""
       for row,row_d6 in enumerate(self.dim_list):
          for col in range(0,3):
             col_d6 = self.dim_list[col]
             xfm_bool = xform_bools[row][col]
             assignment = ""
             assignment = assignment+prefix_in+row_d6+"_"+col_d6
             asn_str = ""
             if (xfm_bool):
                asn_str = asn_str+prefix+row_d6+"_"+col_d6+" <= "+assignment+";"
                if (first_row_bool):
                   mat_str = mat_str+indent+asn_str
                else:
                   mat_str = mat_str+"\n"+indent+asn_str
                first_row_bool = False
             else:
                mat_str = mat_str
       return mat_str

    def genXformMatString(self,prefix,xform_bools):
       xform_mat_strs = []
       first_row_bool = True
       mat_str = ""
       for row,row_d6 in enumerate(self.dim_list):
          for col in range(0,3):
             col_d6 = self.dim_list[col]
             xfm_bool = xform_bools[row][col]
             if (xfm_bool):
                elem_str = prefix+row_d6+"_"+col_d6
                xform_mat_strs.append(elem_str)
       return xform_mat_strs

    #-------------------------------------------------------------------------------

    # Make Transform Input Port
    def makeXformInputPort(self,xform_bools):
       mat_str = ""
       mat_str = mat_str+"   input  signed[(WIDTH-1):0]"+"\n"
       xfm_str = ""
       xfm_str = self.makeXformMatString("xform_in_",xform_bools,"               ","      ")
       mat_str = mat_str+xfm_str
       mat_str = mat_str+","+"\n"
       return mat_str

    # Make Transform Output Port
    def makeXformOutputPort(self,xform_bools):
       mat_str = ""
       mat_str = mat_str+"   output signed[(WIDTH-1):0]"+"\n"
       xfm_str = ""
       xfm_str = self.makeXformMatString("xform_out_",xform_bools,"                ","      ")
       mat_str = mat_str+xfm_str
       mat_str = mat_str+"\n"
       return mat_str

    # Make Transform Wires
    def makeXformWires(self,xform_bools):
       mat_str = ""
       mat_str = mat_str+"   wire signed[(WIDTH-1):0]"+"\n"
       xfm_str = ""
       xfm_str = self.makeXformMatString("xform_",xform_bools,"            ","      ")
       mat_str = mat_str+xfm_str
       mat_str = mat_str+";"+"\n"
       return mat_str

    # Make Transform Xgens Port Assignments
    def makeXformXgensPortAssignments(self,xform_bools):
       mat_str = ""
       xfm_str = ""
       xfm_str = self.makeXformMatPortAssignmentString("xform_out_","xgens_out_",xform_bools,"                                  ","      ")
       mat_str = mat_str+xfm_str
       mat_str = mat_str+"\n"
       return mat_str

    # Make Transform Xgens Port Assignments with wire prefix specified
    def makeXformXgensPortAssignmentsWirePrefix(self,xform_bools, prefix):
       mat_str = ""
       xfm_str = ""
       # todo: variable indent length according to prefix
       xfm_str = self.makeXformMatPortAssignmentString("xform_out_",prefix,xform_bools,"                                  ","      ")
       mat_str = mat_str+xfm_str
       mat_str = mat_str+"\n"
       return mat_str

    # Make Transform Xgens wires with wire prefix specified
    def makeXformWiresPrefix(self,xform_bools,prefix):
       mat_str = ""
       mat_str = mat_str+"   wire signed[(WIDTH-1):0]"+"\n"
       xfm_str = ""
       xfm_str = self.makeXformMatString(prefix,xform_bools,"            ","      ")
       mat_str = mat_str+xfm_str
       mat_str = mat_str+";"+"\n"
       return mat_str

    # Make Transform S1 Muxes
    def makeXformS1Muxes(self,xform_bools):
       mat_str = ""
       xfm_str = ""
       xfm_str = self.makeXformMat1CondAssignString("xform_","s1_bool","xgens_out_","xform_in_",xform_bools,"   ")
       mat_str = mat_str+xfm_str
       mat_str = mat_str+"\n"
       return mat_str

    # Make Transform Xdot Port Assignments
    def makeXformXdotPortAssignments(self,xform_bools):
       mat_str = ""
       xfm_str = ""
       xfm_str = self.make6x6XformMatPortAssignmentString("xform_in_","xform_",xform_bools,"                             ","      ")
       mat_str = mat_str+xfm_str
       mat_str = mat_str+","+"\n"
       return mat_str

    # Make Transform Output Assignments
    def makeXformOutputAssignments(self,xform_bools):
       mat_str = ""
       xfm_str = ""
       xfm_str = self.makeXformMatAssignmentString("xform_out_","xform_",xform_bools,"   ")
       mat_str = mat_str+xfm_str
       mat_str = mat_str+"\n"
       return mat_str

    # Make Transform Output Wires
    def makeXformOutputWires(self,xform_bools):
       mat_str = ""
       mat_str = mat_str+"   wire signed[(WIDTH-1):0]"+"\n"
       xfm_str = ""
       xfm_str = self.makeXformMatString("xform_out_",xform_bools,"                ","      ")
       mat_str = mat_str+xfm_str
       mat_str = mat_str+";"+"\n"
       return mat_str

    # Make Transform Input Port Assignments
    def makeXformInputPortAssignment(self,xform_bools):
        mat_str = ""
        xfm_str = ""
        xfm_str = self.makeXformMatPortAssignmentString("xform_in_","xform_in_",xform_bools,"                                ","      ")
        mat_str = mat_str+xfm_str
        mat_str = mat_str+","+"\n"
        return mat_str

    # Make Transform Output Port Assignments
    def makeXformOutputPortAssignment(self,xform_bools):
        mat_str = ""
        xfm_str = ""
        xfm_str = self.makeXformMatPortAssignmentString("xform_out_","xform_out_",xform_bools,"                                  ","      ")
        mat_str = mat_str+xfm_str
        mat_str = mat_str+"\n"
        return mat_str

    #-------------------------------------------------------------------------------

    def gen_dim_signals(self, prefix):
        dim_list = self.dim_list

        dim_signals = []
        for r in range(6):
            for c in range(6):
                dim_signal = "{prefix}_{row_dim}_{col_dim}".format(prefix=prefix, \
                        row_dim=dim_list[r], col_dim=dim_list[c])
                dim_signals.append(dim_signal)
        return dim_signals

    def gen_xform_dim_signals(self, prefix, xform_bools):
        dim_list = self.dim_list

        dim_signals = []
        for r in range(6):
            for c in range(6):
                dim_signal = None
                #   E   0
                # -Erx  E
                if c < 3:
                    dim_signal = "{prefix}_{row_dim}_{col_dim}".format(prefix=prefix, \
                            row_dim=dim_list[r], col_dim=dim_list[c])
                elif r < 3 and c >= 3:
                    dim_signal = "0"
                else:
                    dim_signal = "{prefix}_{row_dim}_{col_dim}".format(prefix=prefix, \
                            row_dim=dim_list[r-3], col_dim=dim_list[c-3])

                dim_signals.append(dim_signal)

        return dim_signals

