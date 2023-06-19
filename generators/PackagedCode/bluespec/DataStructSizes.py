# Derived from structs in <rbd-accelerator>/PackagedCode/dynamics/FPGATypes.h

####### UTILITY FUNCTIONS ############

def to_bits(bytes):
    return bytes * 8

def to_bytes(bits):
    return bits / 8

def ceiling(x, y):
    return (x + y - 1) // y

def FPGAKnotIn_zero_padding(num_links):
    raw_size = (4 + num_links)*4 * num_links
    bus_width_bytes = 32
    smallest_mul_N = ceiling(raw_size, bus_width_bytes)
    zero_padding_in_bytes = bus_width_bytes * smallest_mul_N - raw_size
    return zero_padding_in_bytes

def FPGAKnotOut_zero_padding(num_links):
    raw_size = (4*num_links) * 2 * num_links
    bus_width_bytes = 32
    smallest_mul_N = ceiling(raw_size, bus_width_bytes)
    zero_padding_in_bytes = bus_width_bytes * smallest_mul_N - raw_size
    return zero_padding_in_bytes

######################################

def get_FPGALinkIn_size_bytes(num_links):
    sinq_size = 4
    cosq_size = 4
    qd_size   = 4
    qdd_size  = 4
    minv_size = num_links * 4
    return sinq_size + cosq_size + qd_size + qdd_size + minv_size

def get_FPGAKnotIn_size_bytes(num_links):
    links_size = num_links * get_FPGALinkIn_size_bytes(num_links)
    zero_padding_size = FPGAKnotIn_zero_padding(num_links)
    return links_size + zero_padding_size

def get_FPGADataIn_size_bytes(num_links, knot_points=10):
    knots_size = knot_points * get_FPGAKnotIn_size_bytes(num_links)
    flag_size = 4
    return knots_size + flag_size

def get_FPGALinkOut_size_bytes(num_links):
    cdq_size = num_links * 4
    cdqd_size = num_links * 4
    return cdq_size + cdqd_size

def get_FPGAKnotOut_size_bytes(num_links):
    links_size = num_links * get_FPGALinkOut_size_bytes(num_links)
    zero_padding_size = FPGAKnotOut_zero_padding(num_links)
    return links_size + zero_padding_size

def get_FPGADataOut_size_bytes(num_links, knot_points=10):
    knots_size = knot_points * get_FPGAKnotOut_size_bytes(num_links)
    flag_size = 4 * 8
    return knots_size + flag_size
