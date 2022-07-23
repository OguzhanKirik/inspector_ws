import open3d as o3d
import sys
from os.path import dirname, abspath
import FrankProfilometer
include_dir = str(dirname(dirname(abspath(__file__)))) + '/include/'
import FrankCommons
import FrankUtilities
sys.path.append(include_dir)
#from REGISTRATION import PC_registration
#from REGISTRATION.ICP_utils import *
from UTILS.pcd_numpy_utils import *
from UTILS.CalibrationUtils import *
import numpy as np
from sklearn.decomposition import PCA
from STEP_ANALYSIS.utils_STEP import *
import copy
import random
import Frank3DCamera


cam = Frank3DCamera.new3DCamera()

prof = FrankProfilometer.newProfilometer()


filepath = "/home/c301/Desktop/TEMP/X36/0"

EX =  -0.85
EY =0.2
EZ = 183.5
R_EX = math.radians(EX)
R_EY = math.radians(EY)
R_EZ = math.radians(EZ)
CoordSyst  = FrankUtilities.Euler2RotMatrix([R_EZ, R_EY, R_EX])
Offset = [89.75, 55 , 40.097]

for i in range(10):
    points_cam = PCDToNumpy(o3d.io.read_point_cloud(filepath+"/3dCam_poincloud"+str(i)+".ply"))
    coords = ReadRobotCoordinateInputFile(filepath+"/Coords"+str(i+1)+".txt")[1]
    quats = ReadRobotCoordinateInputFile(filepath+"/Coords"+str(i+1)+".txt")[2]
    points = cam.Build_point_cloud_3dcam(points_cam, coords, quats, CoordSyst, Offset, SavePath = filepath+"/PointCloud"+str(i+1)+".pcd" )

    