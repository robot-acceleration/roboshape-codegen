import numpy as np
import sympy as sp
from .XtDotMinvGen import XtDotMinvGen
from .FProcGen import FProcGen
from .BProcGen import BProcGen
from .BluespecInterfaceGen import BluespecInterfaceGen
from .ScheduleGen import ScheduleGen
np.set_printoptions(precision=4, suppress=True, linewidth = 100)

"""

"""
class FPGACodegen:
    def __init__(self, robotObj):
        self.robot = robotObj
        self.Xmats = self.robot.get_Xmats_ordered_by_id()
        self.Imats = self.robot.get_Imats_ordered_by_id()

    ##############
    #    XMAT    #
    ##############

    def left_half_matrix(self, mat):
        # only need left half of transformation matrix for
        # X matrix related ops because we can derive 
        # the right half from the left half:
        #   E  0
        # -Erx E
        # featherstone eq 2.24
        return mat[:,:3]

    def get_Xmat_sparsity_boolean_matrix_by_id(self, jid):
        Xmat = self.Xmats[jid]
        bool_mat = sp.Matrix(Xmat.shape[0], Xmat.shape[1],
                lambda i,j: not Xmat[i, j].equals(0))
        return bool_mat

    def get_Xmat_sparsity_boolean_matrix_OR(self):
        # outputs an "aggregate" boolean matrix of all the joint X matrices
        # An entry in `bool_mat_OR` is marked `False` only if
        # every corresponding entry in every X matrix is 0.
        # Conversely, an entry is marked `True` if even one corresponding entry in all the X matrices is non-zero.
        # This is a conservative bound of the amount of sparsity present in the X matrices.
        # Maybe more to be exploited?
        init_bool_mat = self.get_Xmat_sparsity_boolean_matrix_by_id(0)
        bool_mat_OR = np.zeros(init_bool_mat.shape, dtype=bool)
        for jid, Xmat in enumerate(self.Xmats):
            if Xmat is not None:
                bool_mat_jid = self.get_Xmat_sparsity_boolean_matrix_by_id(jid)
                bool_mat_jid = np.array(bool_mat_jid.tolist()).astype(bool)
                bool_mat_OR = np.logical_or(bool_mat_OR, bool_mat_jid)

        return bool_mat_OR

    ##############
    #    IMAT    #
    ##############

    def get_Imat_sparsity_boolean_matrix_by_id(self, lid):
        # entry marked true if != 0
        # else marked false
        Imat = self.Imats[lid]
        epsilon = 1e-5
        # floating pt equality check against zero
        bool_mat = [[abs(Ientry) > epsilon for Ientry in Iarr] for Iarr in Imat]
        return bool_mat

    def get_Imat_sparsity_boolean_matrix_OR(self):
        init_bool_mat = self.get_Imat_sparsity_boolean_matrix_by_id(0)
        bool_mat_OR = np.zeros((6, 6), dtype=bool)
        for lid, Imat in enumerate(self.Imats):
            if Imat is not None:
                bool_mat_lid = self.get_Imat_sparsity_boolean_matrix_by_id(lid)
                bool_mat_lid = np.array(bool_mat_lid).astype(bool)
                bool_mat_OR = np.logical_or(bool_mat_OR, bool_mat_lid)

        return bool_mat_OR

    ### todo: these two functions should be moved to 
    # verilog/bluespec constructs because they have nothing to do
    # with Xmats and Imats
    def decimalToBinaryWithoutSign(self, num):
        WIDTH = 32
        DECIMAL_BITS = 16

        decimal_repr = round(num*(2**DECIMAL_BITS))
        return decimal_repr

    def to_decimal_string_repr(self, num):
        decimal_repr = self.decimalToBinaryWithoutSign(num)
        str_repr = ""
        if decimal_repr < 0:
            str_repr = "-32'd" + str(int(abs(decimal_repr)))
        else:
            str_repr = "32'd" + str(int(decimal_repr))
        return str_repr
    ###

    def construct_inertia_decimal_repr(self):
        # specifically for the way idots has been implemented
        Imat_dec_repr_list = []

        # we only use effective links in idots
        # omitting lid -1 since that's the fixed base
        # todo radhika: is it a flawed asumption that we don't want the fixed base?
        # will there always a fixed base?
        for Imat in self.Imats[1:]:
            Imat_dec_repr = [[self.to_decimal_string_repr(Ientry) for Ientry in Iarr] for Iarr in Imat]
            Imat_dec_repr_list.append(Imat_dec_repr)
        return Imat_dec_repr_list

    ##############################
    #  GET HARDWARE GEN MODULES  #
    ##############################

    def get_xtdotminv(self, vw, block_size):
        num_links = self.robot.get_num_links_effective()
        xform_bools = self.get_Xmat_sparsity_boolean_matrix_OR()
        # changed for block_minv multiply, doesn't affect internal impl
        return XtDotMinvGen(vw, block_size, xform_bools)

    def get_fproc(self, num_PEs):
        num_links = self.robot.get_num_links_effective()
        return FProcGen(num_links, num_PEs)

    def get_bproc(self, num_PEs, block_size):
        num_links = self.robot.get_num_links_effective()
        return BProcGen(num_links, num_PEs, block_size)

    def get_bluespec_interface(self, num_PEs, block_size):
        #return BluespecInterfaceGen(num_PEs, block_size, self.robot)
        return ScheduleGen(self.robot, num_PEs, num_PEs, block_size)

    def get_schedule_gen(self, num_fproc_PEs, num_bproc_PEs, block_size):
        return ScheduleGen(self.robot, num_fproc_PEs, num_bproc_PEs, block_size)
