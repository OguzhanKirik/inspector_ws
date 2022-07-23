from ast import Num
from ensenso_nxlib import NxLibItem, NxLibException,NxLibCommand
from ensenso_nxlib.constants import *
import ensenso_nxlib.api as api
import  open3d as o3d
import numpy as np
import copy
import sys
from pathlib import Path
include_dir = str(Path(__file__).resolve().parent)
sys.path.append(include_dir)
utils_dir = str(Path(__file__).resolve().parent.parent.parent)
sys.path.append(utils_dir)
from UTILS import FrankUtilities
import math
import time
from UTILS.pcd_numpy_utils import * 
import json
class new3DCamera():

    Connected = False
    camera_serial = "218131"
    Filepath='/home/c301/Desktop/TEMP/SPARSE'
    Calibpath = "/home/c301/Desktop/TEMP/CALIB_3DCAM"

 

    cameras = None
    camera = None
    cam_id = None
    SparseScanPointclouds = []
    SparseScanOn = False
    SparseScanPoses = [[1023.82, 481.52, 1478.83, 0.185743, 0.0377529, 0.972604, -0.134593],  # LEFT
                        [1279.69, 597.68, 1478.74, 0.298767, 0.0057794, 0.951244, -0.0764203], # LEFT1
                        [1083.58, -54.72, 1478.99, 0.18477, -0.026058, 0.982349, -0.0130891],  #UPUP
                        [1290.21, -54.70, 1436.44, 0.321542, -0.0276414, 0.946446, -0.00927086],   #UPUP1
                        [700.75, -55.80, 1202.33, 0.213801, -0.00773447, 0.976781, -0.0113359],  #UPDOWN
                        [895.92, -773.55, 1375.55, 0.172134, -0.0026897, 0.978266, 0.11558],  #RIGHT
                        [1028.21, -773.50, 1375.46, 0.295515, 0.0120785, 0.948312, 0.115017]] #RIGHT1

    CalibPoses =        [[1122.1,300 , 841.32, 0.02244, 0.04116, -0.99878, 0.01535],
                        [1122.18, 447.63, 841.43, 0.20442, 0.6775, -0.6727, 0.21608],
                        [1250, -325.65, 841.6, 0.14737, -0.74843, -0.63548, -0.11959],
                        [1531.15, 62.17, 747.74, 0.01983, 0.98229, 0.0089, -0.1861],
                        [930.38, 62.1, 747.93, 0.01322, 0.98797, 0.01355, 0.15349]]
    num_foto=  0
    config_file = "/home/c301/Desktop/TEMP/new_calib.json"
    def __init__(self, Connected = False):
        self.Connected = False

    def Connect(self,wanted_serial="218131"):
        if self.Connected == False:
            print("CONNECTION TO CAMERA3D")
            # Waits for the cameras to be initialized
            api.initialize()
            # References to the root of the nxLib tree
            root = NxLibItem()

            # Reference to the serials subnode of all cameras
            self.cameras = root[ITM_CAMERAS][ITM_BY_SERIAL_NO]
            print(self.cameras.count())
           
            for i in range(self.cameras.count()):
                try:
                    if self.cameras[i][ITM_STATUS][ITM_OPEN].exists():
                        is_available = self.cameras[i][ITM_STATUS][ITM_AVAILABLE].as_bool()
                        serial = self.cameras[i].name()
                        print("Camera with serial {} is currently {}".format(serial, "available" if is_available else "closed"))
                        if serial!= wanted_serial:
                            print("another serial requested")
                            
                        else:
                            print("requested serial found")
                            self.camera_serial = serial
                            cmd = NxLibCommand(CMD_OPEN)
                            cmd.parameters()[ITM_CAMERAS] = self.camera_serial
                            cmd.execute()
                            self.Connected = True
                            self.camera = root[ITM_CAMERAS][ITM_BY_SERIAL_NO][str(self.camera_serial)]
                            if self.camera_serial == "171827":
                                self.LoadConfigFile()
                                self.EX =  -0.85
                                self.EY =0.2
                                self.EZ = 183.5
                                self.R_EX = math.radians(self.EX)
                                self.R_EY = math.radians(self.EY)
                                self.R_EZ = math.radians(self.EZ)
                                self.CoordSyst  = FrankUtilities.Euler2RotMatrix([self.R_EZ,self.R_EY, self.R_EX])
                                self.Offset = [89.75, 55 , 40.097]
                            if self.camera_serial == "218131":
                                self.EX = 0.4554
                                self.EY = 0.0507
                                self.EZ = -89.9
                                self.R_EX = math.radians(self.EX)
                                self.R_EY = math.radians(self.EY)
                                self.R_EZ = math.radians(self.EZ)
                                self.CoordSyst  = FrankUtilities.Euler2RotMatrix([self.R_EZ, self.R_EY, self.R_EX])
                                self.Offset = [-42.005, 53.117 , 2.038]
                            else:
                                print("unknown camera serial")
                                return -1
 
                            return 1
                    else:
                        print('no camera available')
                        return -1
                        pass
                except NxLibException as e:
                    print("An NxLibException occured: Error Text: {}".format(e.get_error_text()))
                    #return -1
                except:
                    print("Something bad happenend, that has been out of our control.")
                    #return -1
        else:
            print('3D camera is already connected')
            return 1
    def Disconnect(self):
        if self.Connected == True:
            print("disconneciton from camera 3d")
            try:
                NxLibCommand(CMD_CLOSE).execute()
                self.Connected = False
                print("camera3d disconnected!")
                return 1
            except NxLibException as e:
                print("An NxLibException occured: Error Text: {}".format(e.get_error_text()))

                return -1
            except:
                print("Something bad happenend, that has been out of our control.")
                return -1
        else:
            return -1
    def LoadConfigFile(self):
        
        cmdopen = NxLibCommand(CMD_OPEN)
        cmdopen.parameters()[ITM_CAMERAS] = self.camera_serial
        cmdopen.execute()
        file = open(self.config_file)
        data = json.dumps(json.load(file))
        print("data")
        tmp = NxLibItem("/tmp")
        #or d in data:
        tmp.set_json(data)
        if(tmp[ITM_PARAMETERS].exists()):
            self.camera[ITM_PARAMETERS].set_json(tmp[ITM_PARAMETERS].as_json(), True)
            print("CONFIG FILE UPLOADED")
        else:
            self.camera[ITM_PARAMETERS].set_json(tmp.as_json(), True)
        
        return
    def Grab3DImage(self,filename=''):
        if self.Connected == True:
        #   # Waits for the cameras to be initialized
        #     #api.initialize()

        #     # References to the root of the nxLib tree
        #     #root = NxLibItem()

        #     # Reference to the serials subnode of all cameras
        #     cameras = root[ITM_CAMERAS][ITM_BY_SERIAL_NO]
            try:
                # if self.cameras[0][ITM_STATUS][ITM_OPEN].exists():
                #     is_available = self.cameras[0][ITM_STATUS][ITM_AVAILABLE].as_bool()
                #     serial = self.cameras[0].name()
                #     print("Camera with serial {} is currently {}".format(serial, "available" if is_available else "closed"))
                #     self.camera_serial = serial
                    # cmd = NxLibCommand(CMD_OPEN)
                    # cmd.parameters()[ITM_CAMERAS] = self.camera_serial
                    # cmd.execute()
                        # Captures with the previous openend camera
                    #api.initialize()
                    # References to the root of the nxLib tree
                    #root = NxLibItem()
                   # cmd = NxLibCommand(CMD_OPEN)
                    #cmd.parameters()[ITM_CAMERAS] = self.camera_serial
                    #md.execute()
                    capture = NxLibCommand(CMD_CAPTURE)
                    capture.parameters()[ITM_CAMERAS] = self.camera_serial
                    capture.execute()

                    # Rectify the the captures raw images
                    rectification = NxLibCommand(CMD_RECTIFY_IMAGES)
                    rectification.execute()

                    # Get the item node of the openend camera
                    camera = get_camera_node(self.camera_serial)
                    # Compute the disparity map
                    disparity_map = NxLibCommand(CMD_COMPUTE_DISPARITY_MAP)
                    disparity_map.execute()

                    # Compute the point map from the disparitu map
                    point_map = NxLibCommand(CMD_COMPUTE_POINT_MAP)
                    point_map.execute()
                    points = NxLibItem()[ITM_CAMERAS][self.camera_serial][ITM_IMAGES][ITM_POINT_MAP].get_binary_data()

                    # Watch the captured point cloud with open3d-python
                    point_cloud = _ensenso_to_open3d(points)
                    frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=500)
                    points = PCDToNumpy(point_cloud)
                    bol = points[:,2]<3000
                    points = points[bol]
                #    # remove side points outside 200mm square
                #     bol = points[:,0]<150
                #     points = points[bol]
                #     bol = points[:,0]>-150
                #     points = points[bol]
                #     bol = points[:,0]<150
                #     points = points[bol]
                #     bol = points[:,0]>-150
                #     points = points[bol]

                    #o3d.visualization.draw_geometries([point_cloud,frame])
                    point_cloud = NumpyToPCD(points)

                    if filename!='':
                        o3d.io.write_point_cloud(filename,point_cloud )
                        print("saving file to : " , filename)
                        #points = PCDToNumpy(point_cloud)

                    self.num_foto+=1

                    return PCDToNumpy(point_cloud)
                    
            except NxLibException as e:
                print("An NxLibException occured: Error Text: {}".format(e.get_error_text()))
                return 0
            except:
                print("Something bad happenend, that has been out of our control.")
                return 0 
        return 0
    def Build_point_cloud_3dcam(self, points_cam,coord,quat, RT, offset, SavePath = ''):
        """
        input: points as (N,3) numpy array, coord as (3,1) numpy array, quat as (4,1) nupy array
        output: points connverted in robot coordinate
        """
        points = copy.deepcopy(points_cam)
        points = (RT@points.T).T + offset #rotation to 6 axis
        quat = np.squeeze(np.asarray(quat))
        RT_rob = np.array(FrankUtilities.Quaternion2RotMatrix(list(quat)))
        points = (RT_rob@points.T).T + coord
        if SavePath!='':
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points)
            o3d.io.write_point_cloud(SavePath, pcd)
        return points



    def SparsePathComputation(self,coord):
        X = []
        Y = []
        Z = []
        EX = []
        EY = []
        EZ = []
        RT = []
        ang = 15
        off = 400
        #view1
        X.append(coord[0]-off-100)
        Y.append(coord[1]-off)
        Z.append(coord[2])
        RT.append(FrankUtilities.Euler2RotMatrix([math.radians(coord[5]),math.radians(coord[4]-ang),math.radians(coord[3]-ang)]))
                #view2
        X.append(coord[0]-off-100)
        Y.append(coord[1]+off)
        Z.append(coord[2])
        RT.append(FrankUtilities.Euler2RotMatrix([math.radians(coord[5]),math.radians(coord[4]+ang),math.radians(coord[3]-ang)]))
                #view2
        X.append(coord[0]+off-100)
        Y.append(coord[1]+off)
        Z.append(coord[2])
        RT.append(FrankUtilities.Euler2RotMatrix([math.radians(coord[5]),math.radians(coord[4]+ang),math.radians(coord[3])]))
        X.append(coord[0]+off-100)
        Y.append(coord[1]-off)
        Z.append(coord[2])
        RT.append(FrankUtilities.Euler2RotMatrix([math.radians(coord[5]),math.radians(coord[4]-ang),math.radians(coord[3])]))
        return [X,Y,Z,RT]





def filter_nans(point_map):
    return point_map[~np.isnan(point_map).any(axis=1)]


def reshape_point_cloud(point_map):
    """
    Reshapes the point cloud array from (m x n x 3) to ((m*n) x 3)
    """
    return point_map.reshape(
        (point_map.shape[0] * point_map.shape[1]), point_map.shape[2])


def get_camera_node(serial):
    root = NxLibItem()  # References the root
    cameras = root[ITM_CAMERAS][ITM_BY_SERIAL_NO]  # References the cameras subnode
    for i in range(cameras.count()):
        found = cameras[i].name() == serial
        if found:
            return cameras[i]
def compute_average_z(point_map):
    z_count = 0
    z_average = 0.0

    point_map = reshape_point_cloud(point_map)
    point_map = filter_nans(point_map)

    for i in range(point_map.shape[0]):
        point = point_map[i]
        z_value = point[2]
        z_average = z_value
        z_count += 1

    if z_count != 0:
        z_average = z_average / z_count
    return z_average

def _ensenso_to_open3d(ensenso_pc):
    point_cloud = o3d.geometry.PointCloud()

    # Reshape from (m x n x 3) to ( (m*n) x 3)
    vector_3d_vector = ensenso_pc.reshape(
        (ensenso_pc.shape[0] * ensenso_pc.shape[1]), ensenso_pc.shape[2])

    # Filter nans: if a row has nan's in it, delete it
    vector_3d_vector = vector_3d_vector[~np.isnan(
        vector_3d_vector).any(axis=1)]
    point_cloud.points = o3d.utility.Vector3dVector(vector_3d_vector)
    return point_cloud
