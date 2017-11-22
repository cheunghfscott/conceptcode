#!/usr/bin/python 	

import numpy as np
from numpy.linalg import inv 
from numpy.linalg import pinv
from sympy import *

a=np.array([[1.,2.],[3.,4.]])
b=inv(a)

print(b)
print(np.transpose(b))
c=np.array([[1,-1,0],[0,1,1]])
d=pinv(c)
print(d)

x,y=symbols("x y")
expr= cos(x)+y
print(expr.subs([(x,2.),(y,2.)]))
