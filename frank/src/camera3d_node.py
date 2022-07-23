#!/usr/bin/env python3
#from catkin_ws.src.UTILS.pcd_numpy_utils import NumpyToPCD
import rospy
import numpy as np
from std_msgs.msg import Bool, Int32, Int16
from sensor_msgs.msg import Image, PointCloud2 
import sys
from pathlib import Path
include_dir = str(Path(__file__).resolve().parent.parent)
sys.path.append(include_dir)
from include import Frank3DCamera
from pathlib import Path
utils_dir = str(Path(__file__).resolve().parent.parent.parent)
sys.path.append(utils_dir)
#from cv_bridge import CvBridge, CvBridgeError
import cv2
import time
import copy
#from ros_numpy.point_cloud2 import pointcloud2_to_xyz_array
from UTILS.pc2_numpy import pointcloud2_to_xyz_array, array_to_xyz_pointcloud2
from UTILS.pcd_numpy_utils import *
from frank.srv import *
from geometry_msgs.msg import Pose
import open3d as o3d
import random
from pykdtree.kdtree import KDTree
def connect_3d_camera(req):
    response = Connect3DCameraResponse()
    if camera3d.Connected:
        print("3d Camera is already connected")
        response.error = "3d Camera is already connected"
        response.res = 1
        return response
    else:
        print("... Connecting to 3D Camera ...")
        res = camera3d.Connect("218131")
        attempt=0
        
        while res<0 and attempt<10:
            res = camera3d.Connect("218131")
            attempt+=1
            rospy.sleep(0.5)

        if res<0:
            print('...error connecting to camera...')
            response.res = 0
            response.error = "error connecting to 3d camera"
            return response
        else:
            print('...3d camera connected...')
            response.res = 1
            response.error = "connected successfully to 3d camera"
            return response

def disconnect_3d_camera(req):
    response = Disconnect3DCameraResponse()
    if camera3d.Connected == False:
        print("camera 3d is already disconnected")
        response.error = "camera 3d is already discsonnected"
        response.res = 1
        return response
    else:
        print("... Disconnecting from 3D Camera ...")
        res = camera3d.Disconnect()
        
        if res<0:
            print('...error disconnecting from camera...')
            response.res = 0
            response.error = "error disconnecting from camera"
            return response
        else:
            print('...3d camera disconnected...')
            response.res = 1
            response.error = "disconnected successfully from camera"
            return response

def grab_3d_pointcloud(req):
    response = Grab3DPointcloudResponse()
    if camera3d.Connected == False:
        print("you're not connected to the camera")
        response.error = "camera 3d is not connected"
        response.res = 2
        return response
    else:

        print('...capturing 3D snapshot...')
        ExposureTime = 300000
        GainValue = 1500
        time.sleep(0.2)
        camera3d.Filepath =rospy.get_param("FilePath/sparse")
        print("saving sparse scan in : " , camera3d.Filepath)
        res = camera3d.Grab3DImage(filename=camera3d.Filepath+"/3dCam_poincloud"+str(camera3d.num_foto)+".pcd") #camera3d.Filepath+"/3dCam_poincloud"+str(camera3d.num_foto)+".pcd")            #res must be Nx3 array?
        if type(res)!= int:
            camera3d.SparseScanPointclouds.append(res)
            print("snapshot capured with success")     
            #converting in robot cordinates
                     
            # getpos =  rospy.ServiceProxy("GetPose",GetPose)
            # resp = getpos()
            # pose = list(resp.pose[2:])
            # fCoord = open(camera3d.Filepath + "/Coords"+str(camera3d.num_foto)+".txt", "w")
            # fCoord.write("Number\tX [mm]\tY [mm]\tZ [mm]\tqw\tqz\tqy\tqz\n"+
            # str(camera3d.num_foto)+"\t"+str(pose[0])+"\t"+str(pose[1])+"\t"+str(pose[2])+"\t"+str(pose[3])+"\t"+str(pose[4])+"\t"+str(pose[5])+"\t"+str(pose[6]))
            # fCoord.close()

            # camera3d.Build_point_cloud_3dcam(copy.deepcopy(res), np.asarray(pose[0:3]), np.asarray(pose[3:7]),camera3d.CoordSyst, camera3d.Offset, SavePath = camera3d.Filepath+"/PointCloud_sparse"+str(camera3d.num_foto)+".ply")   
                                     
            pc2_msg= array_to_xyz_pointcloud2(res)
            camera_pub.publish(pc2_msg)
            response.pointcloud = pc2_msg
            response.res = 1
            response.error = "pointcloud grabbed with success"
            return response
        else:                                                                  
            print('...error capturing frame...')
            response.error = "error capturing pointlcoud"
            response.res = 0
            return response

# def pointcloud_sub_callback(msg):
#     if camera3d.SparseScanOn ==True:
#         points = pointcloud2_to_xyz_array(msg)
#         #convert in robot poses
#         camera3d.SparseScanPointclouds.append(points)
def poses_sub_callback(msg):
    if camera3d.SparseScanOn == True:
        pose = [msg.position.x, msg.position.x, msg.position.x,
                msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        camera3d.SparseScanPoses.append(pose)

def filter_working_volume(points):
    #o3d.visualization.draw_geometries([NumpyToPCD(points)])
	### Z filtering
    bol = np.greater_equal(points[:,2],50)
    points = points[bol,:]
    bol = np.less_equal(points[:,2],600)
    points = points[bol,:]
    ### X filtering
    bol = np.greater_equal(points[:,0],200)
    points = points[bol,:]
    bol = np.less_equal(points[:,0],2000)
    points = points[bol,:]
    ### Y filtering
    bol = np.greater_equal(points[:,1],-1000)
    points = points[bol,:]
    bol = np.less_equal(points[:,1],1000)
    points = points[bol,:]
    print("Radius outlier removal")
    npoints = 100000
    points = np.asarray(random.sample(list(points), npoints))
    pcd = NumpyToPCD(points)
    # pcd = pcd.uniform_down_sample(every_k_points=5)

    cl, ind = pcd.remove_radius_outlier(nb_points=500, radius=100)
    #print("statistical outlier removal")
    pcd = pcd.select_by_index(ind)
    # cl, ind = pcd.remove_statistical_outlier(nb_neighbors=10,
    #                                                 std_ratio=0.01)
    # pcd = pcd.select_by_index(ind)
    
   # pcd = pcd.uniform_down_sample(every_k_points=3)

    points = PCDToNumpy(pcd)
    # media = np.mean(points, axis=0)
    # distances = [np.linalg.norm(a-media) for a in points]
    # thrs = np.percentile(distances, 99)
    # bol = distances<thrs
    # points = points[bol]

    #o3d.visualization.draw_geometries([pcd])
    # clf = IsolationForest(n_estimators=10, warm_start=True)
    # clf.fit(PCDToNumpy(pcd))
    # points = clf.predict(PCDToNumpy(pcd))
    #o3d.visualization.draw_geometries([NumpyToPCD(points)])

    #display_inlier_outlier(voxel_down_pcd, ind)

    return points

def save_sparse_scan(req):
    #print("ciao")
    response = SaveSparseScanResponse()
    points_rob = []
    camera3d.Filepath = str(rospy.get_param("/FilePath/sparse"))
    print("saving sparse scan in : " , camera3d.Filepath)
    print(len(camera3d.SparseScanPointclouds))
    print(camera3d.num_foto)
    for i in range(0,len(camera3d.SparseScanPointclouds)):
        print(i)
        points = camera3d.SparseScanPointclouds[i]
        pose = camera3d.SparseScanPoses[i]
        points_rob.append(camera3d.Build_point_cloud_3dcam(points, np.asarray(pose[0:3]), np.asarray(pose[3:7]),camera3d.CoordSyst, camera3d.Offset, SavePath = camera3d.Filepath+"/PointCloud_sparse"+str(i)+".pcd"))
        fCoord = open(camera3d.Filepath  + "/Coords"+str(i)+".txt", "w")
        fCoord.write("Number\tX [mm]\tY [mm]\tZ [mm]\tqw\tqz\tqy\tqz\n"+
        str(i)+"\t"+str(pose[0])+"\t"+str(pose[1])+"\t"+str(pose[2])+"\t"+str(pose[3])+"\t"+str(pose[4])+"\t"+str(pose[5])+"\t"+str(pose[6]))
        # fCoord.write(str(i)+"\t"+str(pose[0])+"\t"+str(pose[1])+"\t"+str(pose[2])+"\t"+str(pose[3])+"\t"+str(pose[4])+"\t"+str(pose[5])+"\t"+str(pose[6]))
        fCoord.close()
    sparse_scan_complete = np.concatenate(points_rob,axis=0)
    sparse_scan_complete = filter_working_volume(sparse_scan_complete)
    print("saving PointCloudALL to " + camera3d.Filepath + "/PointCloudALL.pcd")
    #o3d.visualization.draw_geometries([NumpyToPCD(sparse_scan_complete)])
    o3d.io.write_point_cloud(camera3d.Filepath + "/PointCloudALL.pcd", NumpyToPCD(sparse_scan_complete))
    #response.pointcloud = array_to_xyz_pointcloud2(sparse_scan_complete)
    camera3d.SparseScanPointclouds = []   #reinizialize point and poses
    response.error = "sparse scan saved successfully"
    response.res = 1
    camera3d.num_foto = 0
    print(len(camera3d.SparseScanPointclouds))
    print(camera3d.num_foto)

    return response
def save_calib_scan(req):
    #print("ciao")
    response = SaveCalibScanResponse()
    points_rob = []

    for i in range(len(camera3d.CalibPoses)):
        points = camera3d.SparseScanPointclouds[i]
        pose = camera3d.CalibPoses[i]
        points_rob.append(camera3d.Build_point_cloud_3dcam(points, np.asarray(pose[0:3]), np.asarray(pose[3:7]),camera3d.CoordSyst, camera3d.Offset, SavePath = camera3d.Calibpath+"/PointCloud_"+str(i)+".pcd"))
        fCoord = open(camera3d.Calibpath  + "/Coords"+str(i)+".txt", "w")
        fCoord.write("Number\tX [mm]\tY [mm]\tZ [mm]\tqw\tqz\tqy\tqz\n"+
        str(i)+"\t"+str(pose[0])+"\t"+str(pose[1])+"\t"+str(pose[2])+"\t"+str(pose[3])+"\t"+str(pose[4])+"\t"+str(pose[5])+"\t"+str(pose[6]))
        #fCoord.write(str(i)+"\t"+str(pose[0])+"\t"+str(pose[1])+"\t"+str(pose[2])+"\t"+str(pose[3])+"\t"+str(pose[4])+"\t"+str(pose[5])+"\t"+str(pose[6]))
        fCoord.close()
    sparse_scan_complete = np.concatenate(points_rob,axis=0)
    o3d.visualization.draw_geometries([NumpyToPCD(sparse_scan_complete)])
    
    #o3d.io.write_point_cloud(camera3d.Filepath + "SparsePointCloudALL.pcd", NumpyToPCD(sparse_scan_complete))
    #response.pointcloud = array_to_xyz_pointcloud2(sparse_scan_complete)
    camera3d.SparseScanPointclouds = []   #reinizialize point and poses
    response.error = "calib scan saved successfully"
    response.res = 1
    camera3d.num_foto = 0
    #camera3d.SparseScanPoses = [] #empty the pointcloud  acquired
    return response


# def current_state(msg):
#     #when a new procedure requested
#     print("current state requested is : ",msg.data)
#     camera3d.num_foto=0
#     if msg.data == -2:
#         #sparse scan requested
#         camera3d.Filepath = "/home/c301/Desktop/TEMP/SPARSE_SCAN/"
#     if msg.data ==-10:
#         #3d cam calibration
#         camera3d.Filepath = "/home/c301/Desktop/TEMP/CALIB_3DCAM/"

if __name__ == '__main__':

    try:
        camera3d = Frank3DCamera.new3DCamera()
        rospy.init_node("camera_3d_node")
        s_conn = rospy.Service("connect_3dcamera", Connect3DCamera, connect_3d_camera)
        s_disconn = rospy.Service("disconnect_3dcamera", Disconnect3DCamera, disconnect_3d_camera)
        camera_pub = rospy.Publisher("camera3d_pointcloud",PointCloud2, queue_size=1)
        #camera_sub = rospy.Subscriber("camera3d_pointcloud",PointCloud2, pointcloud_sub_callback)
        send_sparse = rospy.Service("save_sparse_pointcloud",SaveSparseScan, save_sparse_scan) 
        save_calib = rospy.Service("save_calib_pointcloud",SaveCalibScan, save_calib_scan)
        #state_sub = rospy.Subscriber("/state_request",Int16, current_state)   
        s_grab = rospy.Service("grab_3dpointcloud", Grab3DPointcloud, grab_3d_pointcloud)
        print("CAMERA3D is ready to serve")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
