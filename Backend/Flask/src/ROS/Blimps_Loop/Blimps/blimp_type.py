# ========== Blimp Type ========== #

"""
Description:

Check if Blimp Type is an Attack Blimp.

"""

from .blimp_names import attack_blimp_names

def is_attack_blimp(blimp_name):
    attack_blimp = True if blimp_name in attack_blimp_names else False   
    return attack_blimp
