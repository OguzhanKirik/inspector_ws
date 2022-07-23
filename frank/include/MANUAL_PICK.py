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
import numpy as np
from sklearn.decomposition import PCA
from STEP_ANALYSIS.utils_STEP import *
import copy
import random
def pick_points(pcd):
    print("")
    print(
        "1) Please pick at least three correspondences using [shift + left click]"
    )
    print("   Press [shift + right click] to undo point picking")
    print("2) Afther picking points, press q for close the window")
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window()
    vis.add_geometry(pcd)
    vis.run()  # user picks points
    vis.destroy_window()
    print("")
    return vis.get_picked_points()

filepath = "/home/c301/Desktop/TEMP/DENSE"
print("combining all pointcloud")
    ## build the list of all available dense scan
num_ls = []
for i in range(2000):
    if os.path.isfile(filepath+'/PointCloud_'+str(i)+'.pcd'):
        num_ls.append(i)
print(num_ls)
all_pointcloud = []
for i in num_ls:
    print(i)
    dense0_pcd = o3d.io.read_point_cloud(filepath+'/PointCloud_'+str(i)+'.pcd')
    dense0 = PCDToNumpy(dense0_pcd)
    bol = dense0[:,2] >100
    dense0 = dense0[bol]
    #dense0 = FilterWorkingVolume(dense0)
    all_pointcloud.append(dense0)
    
DensePointcloud = np.concatenate(all_pointcloud,axis=  0)
# #downsample random to 2 milion points
# #DensePointcloud = np.asarray(random.sample(list(DensePointcloud), 10000000))
o3d.io.write_point_cloud(filepath+'/DensePointCloudALL.pcd',NumpyToPCD(DensePointcloud))

#filepath = "/home/c301/Desktop/TEMP/DENSE"
# nums = [13,1013]
# [XX,YY,ZZ] = profi.ReadPointCloudInputTextFile(filepath+"/PointCloud_"+str(num)+".txt")
# profi.BuildPointCloud_RobotCoordinate(Num, Coord, Quat, XX, YY, ZZ, SavePath=""))
pcd1 = o3d.io.read_point_cloud(filepath+ "/DensePointCloudALL.pcd")

pcd1.estimate_normals(
    search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=5, max_nn=50))
    
pcd2 =copy.deepcopy(pcd1)
pcd3 = copy.deepcopy(pcd1)
#pcd4 = copy.deepcopy(pcd1)
# pcd1 = o3d.io.read_point_cloud(filepath+ "/PointCloud_1001.pcd")
# pcd2 = o3d.io.read_point_cloud(filepath+ "/PointCloud_1.pcd")
# pt1 = PCDToNumpy(pcd1)
# pt2 = PCDToNumpy(pcd2)
# pts = np.concatenate([pt1,pt2], axis=0)
# #o3d.visualization.draw_geometries([pcd])
# pcd = NumpyToPCD(pts)
picked1= pick_points(pcd1)
picked2= pick_points(pcd2)
picked3= pick_points(pcd3)
##picked4= pick_points(pcd4)


pcd1 = pcd1.select_by_index(picked1)
pcd2 = pcd2.select_by_index(picked2)
pcd3 = pcd3.select_by_index(picked3)
#pcd4 = pcd4.select_by_index(picked4)

points1 = PCDToNumpy(pcd1)
points2 = PCDToNumpy(pcd2)
points3 = PCDToNumpy(pcd3)
#points4 = PCDToNumpy(pcd4)


quat = np.asarray([0,1,0,0])
poses = []

pca=PCA(n_components=1)
pca.fit(points1)
vector1 = pca.components_[0]

pca=PCA(n_components=1)
pca.fit(points2)
vector2 = pca.components_[0]

pca=PCA(n_components=1)
pca.fit(points3)
vector3 = pca.components_[0]

# pca=PCA(n_components=1)
# pca.fit(points4)
# vector4 = pca.components_[0]
pxs=  []
pys = []
pzs = []
pcd = [pcd1,pcd2,pcd3]#,pcd4]
all_points = [points1,points2,points3]#,points4]
vectors = [vector1,vector2,vector3]#,vector4]

for j in range(len(all_points)):
    points1 = all_points[j]
    pcd1 = pcd[j]
    vector1 = vectors[j]
    for i in range(len(points1)):
        normal = pcd1.normals[i]
        sc = np.dot(normal,[0,0,1])
        if sc>0:
            normal = -normal
        #normal =  np.linalg.norm(normal)
        v1 = np.cross(normal,vector1)
        v2 = np.cross(v1,normal)
        #v1 = np.linalg.norm(v1)
        #RT = [list(v2), list(v1),list(normal)]
        RT = FrankCommons.VersorsToRotMatrix(list(v2), list(v1), list(normal))
        quat = FrankUtilities.RotMatrix2Quaternion(RT)
        print(RT)
        print(np.linalg.det(RT))
        draw = []    
                        # allow visualization of the TCP reference frame to check if the computed orientation are correct
        for distance in range(1,10,1):
            px =  points1[i]+distance*v2
            pxs.append(px)
        for distance in range(1,10,1):
            py = points1[i]+distance*v1
            pys.append(py)
    
        for distance in range(1,10,1):
            pz = points1[i]+distance*normal
            pzs.append(pz)
        poses.append([j,points1[i,0],points1[i,1],points1[i,2],quat[0],quat[1],quat[2],quat[3]])

pzs = np.asarray(pzs)  
pys = np.asarray(pys)  
pxs = np.asarray(pxs)  

draw.append(NumpyToPCD(pzs).paint_uniform_color([0,0,1]))
draw.append(NumpyToPCD(pys).paint_uniform_color([0,1,0]))
draw.append(NumpyToPCD(pxs).paint_uniform_color([1,0,0]))
draw.append(NumpyToPCD(points1).paint_uniform_color([0,0,0]))  
o3d.visualization.draw_geometries(draw)
    #poses.append([0,points1[i,0],points1[i,1],points1[i,2],quat[0],quat[1],quat[2],quat[3]])
# for i in range(len(points2)):
#     poses.append([1,points2[i,0],points2[i,1],points2[i,2],quat[0],quat[1],quat[2],quat[3]])
#     v1 = np.cross(normal,vector1)

# for i in range(len(points3)):
#     poses.append([2,points3[i,0],points3[i,1],points3[i,2],quat[0],quat[1],quat[2],quat[3]])
np.savetxt("/home/c301/Desktop/TEMP/DRILL/drill_tcp_pose2.txt", poses)

# o3d.visualization.draw_geometries([pcd1])