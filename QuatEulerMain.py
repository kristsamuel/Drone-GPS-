
#This main file demonstrates functions for handling
#and manipulating quaternios and Euler Angles


from numpy import *
import numpy as np
from QuatEulerFunctions import *

# values for demonstration purposes
q_ = np.array([0.25,0.5,0.1,0.2])

print ('Quaternion: ')
print (q)

rpy_ = quat2rpy(q_)
print ('Euler angles')
print (rpy)_

quat_ = rpy2quat(rpy_)
print ('Recovered quaternion:')
print (quat_)

rot_ = quat2r(quat_)
print ('Recovered rotation matrix:')
print (rot_)

rpy_rec_ = r2rpy(rot_)
print ('Recovered Euler')
print (rpy_rec_)

q_norm_ = normalized(q_)
print ('Normalized quaternion:')
print (q_norm_)