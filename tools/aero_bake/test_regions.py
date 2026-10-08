import unittest
import numpy as np
from scipy.spatial import cKDTree
from prepare_regions import classify,STL

class RegionTests(unittest.TestCase):
    def test_stationary_seam_and_distant_points_do_not_rotate(self):
        trees=[cKDTree([[0,0,0]])]+[cKDTree([[i,0,0]]) for i in range(1,5)]
        labels,_=classify(np.array([[0,0,0],[1,0,0],[2,0,0],[3,0,0],[4,0,0],[.5,0,0],[10,0,0]]),trees,.01)
        self.assertEqual(labels.tolist(),[0,1,2,3,4,0,0])
    def test_binary_records_keep_standard_stl_stride(self):
        self.assertEqual(STL.itemsize,50)

if __name__=='__main__':unittest.main()
