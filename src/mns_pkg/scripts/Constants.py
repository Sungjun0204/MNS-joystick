# B
Bh = 0
Buy = 0
Buz = 0
B_m = 0
g_m = 0
B_g = 0
g_g = 0

# Coil constants
k_h  = 1.6146
k_uy = 1.6149
k_uz = 1.6174
k_m  = 5.4125
k_g  = 6.6220

# Resistance
Rh  = 8.4
Ruy = 11.0
Ruz = 9.2
Rm  = 6.2
Rg  = 10.4

# Current
Ih  = Bh/k_h
Iuy = Buy/k_uy
Iuz = Buz/k_uz
Im  = g_m/k_m
Ig  = g_g/k_g

# Voltage
Vh  = Ih*Rh
Vuy = Iuy*Ruy
Vuz = Iuz*Ruz
Vm  = Im*Rm
Vg  = Ig*Rg