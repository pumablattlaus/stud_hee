#!/usr/bin/env python
# # coding=latin-1

import numpy as np
from pyquaternion import Quaternion
import tf.transformations as transf

if __name__ == "__main__":
    # q1 = np.array([0, 0, 0.7071068, 0.7071068])
    # q2 = np.array([0, 0, 0.9238795, 0.3826834])
    # q1_inv = np.array([*q1[0:3]*-1, q1[3]])
    # q_delta = q2*q1_inv
    # print(q_delta)

    # Quaternion = w,x,y,z
    q1 = Quaternion(0.7071068, 0, 0, 0.7071068)
    q2 = Quaternion(0.3826834, 0, 0, 0.9238795)
    q1_inv = q1.conjugate
    q_delta = q2 * q1_inv
    print(q_delta)

    q1_ = [0, 0, 0.7071068, 0.7071068]
    q2_ = [0, 0, 0.9238795, 0.3826834]
    q1_inv_ = transf.quaternion_inverse(q1_)
    q1_conj_ = transf.quaternion_conjugate(q1_)
    q_delta_ = transf.quaternion_multiply(q2_, q1_conj_)
    print (q_delta_)

