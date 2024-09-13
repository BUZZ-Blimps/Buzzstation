# ========== Blimp Names ========== #

"""
Description:

Allowed blimp names and the blimp names order are stored here.

To-Do:

- Use a yaml file to read allowed catching and attack blimp names

"""

from Packages.packages import yaml

# Read the YAML file
with open('../src/Config/blimp_names.yaml', 'r') as file:
    blimp_data = yaml.safe_load(file)

# Allowed Catching Blimps
catching_blimp_names = blimp_data.get('catching_blimp_names', [])

# Allowed Attack Blimps
attack_blimp_names = blimp_data.get('attacking_blimp_names', [])

# Order
blimp_names_order = catching_blimp_names + attack_blimp_names
