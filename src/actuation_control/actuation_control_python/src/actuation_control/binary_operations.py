# Author: ML lab continuum-actuation team 
# University of California, San Diego

def uint32_to_int32(x: int) -> int:
    """
    Convert a 32-bit unsigned integer to signed int32.
    """
    x = x & 0xFFFFFFFF  # mask to 32 bits
    if x & 0x80000000:  # if sign bit is set
        return -((~x & 0xFFFFFFFF) + 1)
    else:
        return x
    
def check_bit(num: int, bit: int) -> int:
    """
    Check if a given bit is set in a number.
    num : integer
    bit : bit position (0 = least significant bit)
    
    Returns 1 if the bit is set, 0 otherwise.
    """
    return 1 if (num & (1 << bit)) else 0