import numpy as np
import scipy as sp



class RotationMatrix:


    def __init__(self, mat):
        u, p = sp.linalg.polar(mat)
        self._mat = u
        
        
    def normalize(self):
        self._mat, dummy = sp.linalg.polar(self._mat)
        
        
        
        
        
if __name__ == '__main__':
    mat = np.random.rand(3,3)
    print mat
    rot = RotationMatrix(mat)
    print rot
