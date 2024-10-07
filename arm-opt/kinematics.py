from casadi import *
from constants import *
import numpy as np

def fky(j1, j2):
    y = link1Len * sin(j1) + link2Len * sin(j1 + j2)

    return y

def fkx(j1, j2):
    x = link1Len * cos(j1) + link2Len * cos(j1 + j2)
    return x

def fk1(j1):
    return link1Len*cos(j1), link2Len*sin(j1)
