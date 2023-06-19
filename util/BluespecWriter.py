from .VectorGenerator import VectorGenerator
from .BaseWriter import BaseWriter

from math import log2, floor, ceil

class BluespecWriter(BaseWriter):
    def __init__(self, file, dim_list, num_links, block_size=-1):
        super(BluespecWriter, self).__init__(file, dim_list, num_links, block_size)

    def replaceNumLinksAndWrite(self, line):
        assert "{num_links}" in line, "Must specify {num_links} in format str to replace!"
        self.writeLine(line.format(num_links=self.num_links))

    def replaceNumLinksIncrAndWrite(self, line):
        assert "{num_links_incr}" in line, "Must specify {num_links_incr} in format str to replace!"
        self.writeLine(line.format(num_links_incr=self.num_links+1))

    def replaceNumLinksDecrAndWrite(self, line):
        assert "{num_links_decr}" in line, "Must specify {num_links_decr} in format str to replace!"
        self.writeLine(line.format(num_links_decr=self.num_links-1))

    def vectorListToPorts(self, vec_list):
        res = [vec+"()" for vec in vec_list]
        return res

    def assignVectorLists(self, list1, list2):
        associated_vectors = self.associateVectorLists(list1, list2)
        for lvalue, rvalue in associated_vectors:
            self.file.write(self.indent + "let {lvalue} = {rvalue};\n".format(lvalue=lvalue, rvalue=rvalue))

    def portAssignment(self, port_list, vec_list):
        associated_vectors = self.associateVectorLists(port_list, vec_list)
        for lvalue, rvalue in associated_vectors:
            self.writeLine(self.indent + lvalue + "(" + rvalue + ");")

    def writeLibraryList(self, library_list):
        for lib in library_list:
            self.writeLine("import " + lib + ";")

    def writeLineWithFormatSpecReplace(self, line):
        num_links = self.num_links
        block_size = self.block_size

        num_links_decr = num_links-1
        num_links_incr = num_links+1
        max_link_ref = num_links+1
        max_link_incr = max_link_ref+1
        max_link_counter_bitwidth = int(floor(log2(max_link_ref)))+1
        padded_matrix_dim = int(ceil(num_links / block_size)) * block_size

        fmap = {}
        if "{num_links_decr}" in line:
            fmap["num_links_decr"] = num_links_decr
        if "{num_links}" in line:
            fmap["num_links"] = num_links
        if "{num_links_incr}" in line:
            fmap["num_links_incr"] = num_links_incr
        if "{max_link_ref}" in line:
            fmap["max_link_ref"] = max_link_ref
        if "{max_link_incr}" in line:
            fmap["max_link_incr"] = max_link_incr
        if "{max_link_counter_bitwidth}" in line:
            fmap["max_link_counter_bitwidth"] = max_link_counter_bitwidth
        if "{padded_matrix_dim}" in line:
            fmap["padded_matrix_dim"] = padded_matrix_dim

        self.writeLine(line.format_map(fmap))

    def get_row_vec_bluespec(self, prefix, row):
        row_vec = prefix + "vec(" + ",".join(map(str, row)) + ")"
        return row_vec

    def get_matmul_sched_table_str_bluespec(self, prefix, sched_table):
        sched_table_str = ""
        for i,sched_row in enumerate(sched_table):
            if i == len(sched_table)-1:
                ending = ""
            else:
                ending = ","
            sched_table_str += self.get_row_vec_bluespec(prefix, sched_row) + ending + "\n"
        sched_table_str += ");"
        return sched_table_str

    def get_matmul_sched_table_full_decl_str_bluespec(self, sched_table, sched_table_name, \
            len_block_minv_sched_per_matrix, num_PEs, max_link_counter_bitwidth):

        matmul_decl_str = "Vector#({len_block_minv_sched_per_matrix}, Vector#({num_PEs}, Bit#({max_link_counter_bitwidth}))) {sched_table_name} = vec(\n".format(
            len_block_minv_sched_per_matrix=len_block_minv_sched_per_matrix,
            num_PEs=num_PEs,
            max_link_counter_bitwidth=max_link_counter_bitwidth,
            sched_table_name=sched_table_name
        )

        matmul_decl_str += self.get_matmul_sched_table_str_bluespec("    ", sched_table)
        matmul_decl_str += "\n"

        return matmul_decl_str

