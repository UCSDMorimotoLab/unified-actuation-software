# Author: ML lab continuum-actuation team 
# University of California, San Diego

import numpy as np 
import PyKDL

def convert_np_tf_to_PyKDL_frame(T):
    '''
    '''

    frame = PyKDL.Frame()
    frame.M = PyKDL.Rotation(T[0,0],T[0,1],T[0,2],
                            T[1,0],T[1,1],T[1,2],
                            T[2,0],T[2,1],T[2,2])
    frame.p = PyKDL.Vector(T[0,3],T[1,3],T[2,3])

    return frame 

def convert_PyKDL_frame_to_np_tf(frame):
    '''
    '''
    T = np.eye(4)

    T[:3,:3] = np.array([[frame.M[0,0], frame.M[0,1], frame.M[0,2]],
                        [frame.M[1,0], frame.M[1,1], frame.M[1,2]], 
                        [frame.M[2,0], frame.M[2,1], frame.M[2,2]]])
    T[:3,3] = np.array([frame.p[0],frame.p[1],frame.p[2]])
    
    return T 