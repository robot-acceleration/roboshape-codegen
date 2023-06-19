class VectorGenerator:
    def __init__(self, dim_list, num_links, block_size=-1):
        self.dim_list = dim_list
        self.num_links = num_links
        if block_size == -1:
            self.block_size = num_links
        else:
            self.block_size = block_size
        
        joint_strs = []
        for j in range(0, num_links):
            joint_strs.append("J" + str(j+1))
        self.joint_strs = joint_strs

    def appendDim(self, *prefix):
        vec_list = []
        for p in prefix:
            for dim in self.dim_list:
                vec_list.append(p + "_" + dim)
        return vec_list

    def appendJoint(self, *prefix):
        vec_list = []
        for p in prefix:
            for joint in self.joint_strs:
                vec_list.append(p + "_" + joint)
        return vec_list

    def genRowColVector_1d(self, prefix):
        vec_list = []
        for j_r, joint_r in enumerate(self.joint_strs):
            for j_c, joint_c in enumerate(self.joint_strs):
                vec_list.append("{prefix}_R{r}_C{c}".format(prefix=prefix, r=j_r+1, c=j_c+1))
        return vec_list

    def genColRowVector_1d(self, prefix):
        vec_list = []
        for j_c, joint_c in enumerate(self.joint_strs):
            for j_r, joint_r in enumerate(self.joint_strs):
                vec_list.append("{prefix}_C{c}_R{r}".format(prefix=prefix, c=j_c+1, r=j_r+1))
        return vec_list

    def genColRowVector_1d_block_size(self, prefix):
        vec_list = []
        for c in range(self.block_size):
            for r in range(self.block_size):
                vec_list.append("{prefix}_C{c}_R{r}".format(prefix=prefix, c=c+1, r=r+1))
        return vec_list

    def jointIndexFormatStr(self, start_index, *prefix):
        vec_list = []
        for p in prefix:
            assert "{j}" in p, "Incorrectly formatted prefix!"
            for j, joint in enumerate(self.joint_strs):
                vec_list.append(p.format(j=j+start_index))
        return vec_list

    def dimIndexFormatStr(self, start_index, *prefix):
        vec_list = []
        for p in prefix:
            assert "{d}" in p, "Incorrectly formatted prefix!"
            for d, dim in enumerate(self.dim_list):
                vec_list.append(p.format(d=d+start_index))
        return vec_list

    def dimNameFormatStr(self, *prefix):
        vec_list = []
        for p in prefix:
            assert "{dim}" in p, "Incorrectly formatted prefix!"
            for d, dim in enumerate(self.dim_list):
                vec_list.append(p.format(dim=dim))
        return vec_list

    def blockSizeFormatStr(self, prefix):
        vec_list = []
        for i in range(self.block_size):
            assert "{b}" in prefix, "Incorrectly formatted prefix!"
            vec_list.append(prefix.format(b=i+1))
        return vec_list
