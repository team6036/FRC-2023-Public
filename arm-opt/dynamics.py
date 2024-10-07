import numpy as np
from casadi import *
from constants import *


def ff(theta1, theta2, theta1_dot, theta2_dot, wa1, wa2):
    c2 = cos(theta2)
    hM = link1Len * com2 * c2
    hC = -mass2 * link1Len * com2 * sin(theta2)

    M = MX(2, 2)
    M[0, 0] = mass1*com1*com1 + mass2*(link1Len**2 + com2**2 * 2*hM) + j1Moi + j2Moi
    M[0, 1] = mass2 * (com2**2 + hM) + j2Moi
    M[1, 0] = mass2 * (com2**2 + hM) + j2Moi
    M[1, 1] = mass2 * (com2**2) + j2Moi

    C = MX(2, 2)
    C[0, 0] = hC * theta2_dot
    C[0, 1] = hC * (theta1_dot + theta2_dot)
    C[1, 0] = -hC * theta1_dot
    C[1, 1] = 0

    G = MX(2, 1)
    G[0, 0] = g * cos(theta1) * (mass1 * com1 + mass2 * link1Len) + g * cos(theta1 + theta2) * mass2 * com2
    G[1, 0] = g * cos(theta1 + theta2) * mass2 * com2

    a = MX(2, 1)
    a[0, 0] = wa1
    a[1, 0] = wa2

    v = MX(2, 1)
    v[0, 0] = theta1_dot
    v[1, 0] = theta2_dot

    torque = M @ a + C @ v + G

    print("t", torque)

    return K3_inv @ (torque + K4 @ a)

print("k4", K4)
print("k3", K3)
print("n1", K3_inv[0, 0])
print("n2", K3_inv[1, 1])
print("k3i", K3_inv)

print("f", ff(0, 0, 0, 0, 0, 0))
