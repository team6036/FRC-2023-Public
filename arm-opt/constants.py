import numpy as np
from casadi import *

eff = 0.95

#physical
g = 9.79922 * eff

link1Len = 0.9144
link2Len = 0.7577836

com1 = 0.38735
com2 = 0.2917698

j1GR = (60.0/8) * (50.0/16) * (36.0/12)
j2GR = (56.0/14) * (60.0/26) * (36.0/12) * (36.0/24)

j1Moi = 22159.905 * 1.829e-5
j2Moi = 31780.362 * 1.829e-5

mass1 = 2.84048047
mass2 = 6.484471473

motors1 = 2
motors2 = 2

#motor
stall_torque = 4.69 * eff
stall_current = 257
free_speed = (6380 / 60) * 2 * 3.141592

#calculated
Rm = 12.0 / stall_current
Kv = free_speed / 12.0
Kt = stall_torque / stall_current

K4 = MX(2, 2)
K4[0, 0] = j1GR * j1GR * motors1 * Kt / Kv / Rm
K4[1, 1] = j2GR * j2GR * motors2 * Kt / Kv / Rm

K3 = MX(2, 2)
K3[0, 0] = j1GR * motors1 * Kt / Rm
K3[1, 1] = j2GR * motors2 * Kt / Rm

K3_inv = MX(2, 2)
det = 1.0 / (K3[0, 0] * K3[1, 1])
K3_inv[0, 0] = K3[1, 1] * det
K3_inv[1, 1] = K3[0, 0] * det
