from .VectorGenerator import VectorGenerator

class BaseWriter(object):
    def __init__(self, file, dim_list, num_links, block_size=-1):
        self.file = file
        self.vec_gen = VectorGenerator(dim_list, num_links, block_size)
        self.indent = ""
        self.dim_list = dim_list
        self.num_links = num_links

        if block_size == -1:
            self.block_size = num_links
        else:
            self.block_size = block_size

    def appendDim(self, *prefix):
        return self.vec_gen.appendDim(*prefix)

    def appendJoint(self, *prefix):
        return self.vec_gen.appendJoint(*prefix)

    def indexDim(self, *prefix):
        return self.vec_gen.indexDim(*prefix)

    def indexJoint(self, *prefix):
        return self.vec_gen.indexJoint(*prefix)

    def genRowColVector_1d(self, prefix):
        return self.vec_gen.genRowColVector_1d(prefix)

    def genColRowVector_1d(self, prefix):
        return self.vec_gen.genColRowVector_1d(prefix)

    def genColRowVector_1d_block_size(self, prefix):
        return self.vec_gen.genColRowVector_1d_block_size(prefix)

    def jointIndexFormatStr(self, start_index, *prefix):
        return self.vec_gen.jointIndexFormatStr(start_index, *prefix)
    
    def dimIndexFormatStr(self, start_index, *prefix):
        return self.vec_gen.dimIndexFormatStr(start_index, *prefix)

    def dimNameFormatStr(self, *prefix):
        return self.vec_gen.dimNameFormatStr(*prefix)

    def blockSizeFormatStr(self, prefix):
        return self.vec_gen.blockSizeFormatStr(prefix)

    def setIndentLevel(self, indent):
        self.indent = indent

    def writeCommaSep(self, list, lineSepElements=-1, last=0):
        if last == 0:
            # have more elements following this
            commaEnd = ","
        elif last == 1:
            commaEnd = ""
        elif last == 2:
            commaEnd = ";"

        if lineSepElements == -1:
            # print all the terms in a single line
            self.writeLine(self.indent + ", ".join(list) + commaEnd)
        else:
            assert lineSepElements != 0, "There must be at least one element per line"
            # print lineSepElements list items per line
            chunked_list = [list[i:i + lineSepElements] for i in range(0, len(list), lineSepElements)]
            for c, chunk in enumerate(chunked_list):
                if c < len(chunked_list) - 1:
                    self.writeLine(self.indent + ", ".join(chunk) + ",")
                else:
                    self.writeLine(self.indent + ", ".join(chunk) + commaEnd)

    # use this for assigning a set of vectors to each other
    # list1 = ['a', 'b', 'c']
    # list2 = ['w', 'x', 'y']
    # out   = [('a', 'w'), ('b', 'x'), ('c', 'y')]
    def associateVectorLists(self, *lists):
        assoc = zip(*lists)
        return assoc

    def writeLine(self, line=""):
        self.file.write(line + "\n")

    def writeRaw(self, string):
        self.file.write(string)

    def assignVectorLists(self, list1, list2):
        raise NotImplementedError

    def regUpdateVectorLists(self, list1, list2):
        raise NotImplementedError

    def get_bitwidth(self, num):
        return len(bin(abs(num))) - 2

    def get_bitwidth_array_def(self, num):
        return self.get_bitwidth(num) - 1
