import numpy as np
import warnings as wrn


def normalize(array):
    array = np.array(array)
    norm = np.linalg.norm(array)
    if norm <= 0.01:
        wrn.warn('The norm is too small.')
        wrn.warn('Setting the vector to (1,0).')
        return np.array([1.0, 0.0, 0.0])
    return array/norm


def saturate(array, ths):
    norm = np.linalg.norm(array)
    if norm > ths:
        array *= ths/norm
    return array



def skew(v):
    #if len(v) == 4: v = v[:3]/v[3]
    skv = np.roll(np.roll(np.diag(v.flatten()), 1, 1), -1, 0)
    return skv - skv.T

# def perp(array):
#     x = array[0]
#     y = array[1]
#     return np.array([-y, x])
