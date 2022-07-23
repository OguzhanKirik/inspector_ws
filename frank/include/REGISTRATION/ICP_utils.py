from numpy.core.numeric import count_nonzero
from pyntcloud import PyntCloud 
import numpy as np
from mpl_toolkits.mplot3d import Axes3D 
import matplotlib.pyplot as plt 
import pandas as pd
import os
import sys
import pdb
import sys
from os.path import dirname, abspath
from pathlib import Path
include_dir = str(Path(__file__).resolve().parent)
sys.path.append(include_dir)
include_dir = str(Path(__file__).resolve().parent.parent)
sys.path.append(include_dir)
include_dir = str(Path(__file__).resolve().parent.parent.parent)
sys.path.append(include_dir)
from UTILS.FrankCommons import *
from UTILS.FrankUtilities import *
from UTILS.pcd_numpy_utils import *
from sklearn.decomposition import PCA
#from utils_STEP import *
from PIL import Image
import copy
import random
import cv2
from pykdtree.kdtree import KDTree
import math
global count
count = 0
from scipy.optimize import minimize_scalar
def Edge_Detection(points, k_n, thresh):
    """
    input: points numpy array list, k_n and thresh parameters that influence
            the edge detection
    output: pointcloud of just the edge extracted
    """
    #pcd1 = PyntCloud.from_file("/ArtificialPointClouds/bunny.pcd")
    #pcd1 = PyntCloud.from_file("/TetrahedronMultiple.pcd")
    #pcd1 = PyntCloud.from_file("/ArtificialPointClouds/CubeFractal2.pcd")
    #output_dir = "./detected_edge/"

    #if not os.path.exists(output_dir):
    #    os.makedirs(output_dir)
    clmns = ['x','y','z']
    points_pd = pd.DataFrame(data=points,columns=clmns)
    pcd1 = PyntCloud(points_pd)
    # define hyperparameters


    pcd_np = np.zeros((len(pcd1.points),6))

    # find neighbors
    kdtree_id = pcd1.add_structure("kdtree")
    k_neighbors = pcd1.get_neighbors(k=k_n, kdtree=kdtree_id) 

    # calculate eigenvalues
    ev = pcd1.add_scalar_field("eigen_values", k_neighbors=k_neighbors)

    x = pcd1.points['x'].values 
    y = pcd1.points['y'].values 
    z = pcd1.points['z'].values 

    e1 = pcd1.points['e3('+str(k_n+1)+')'].values
    e2 = pcd1.points['e2('+str(k_n+1)+')'].values
    e3 = pcd1.points['e1('+str(k_n+1)+')'].values

    sum_eg = np.add(np.add(e1,e2),e3)
    sigma = np.divide(e1,sum_eg)
    sigma_value = sigma
    #pdb.set_trace()
    #img = ax.scatter(x, y, z, c=sigma, cmap='jet')

    # visualize the edges
    sigma = sigma>thresh

    #fig = plt.figure()
    #ax = fig.add_subplot(111, projection='3d')

    ## Visualize each one of the eigenvalues
    ##img = ax.scatter(x, y, z, c=e1, cmap='jet')
    ##img = ax.scatter(x, y, z, c=e2, cmap='jet')
    ##img = ax.scatter(x, y, z, c=e3, cmap='jet')

    ## visualize the edges
    #img = ax.scatter(x, y, z, c=sigma, cmap='jet')
    ##img = ax.scatter(x, y, z, c=sigma, cmap=plt.hot())

    #fig.colorbar(img) 
    #plt.show() 

    # Save the edges and point cloud
    thresh_min = sigma_value < thresh
    sigma_value[thresh_min] = 0
    thresh_max = sigma_value > thresh
    sigma_value[thresh_max] = 255

    pcd_np[:,0] = x
    pcd_np[:,1] = y
    pcd_np[:,2] = z
    pcd_np[:,3] = sigma_value

    edge_np = np.delete(pcd_np, np.where(pcd_np[:,3] == 0), axis=0) 

    clmns = ['x','y','z','red','green','blue']
    pcd_pd = pd.DataFrame(data=pcd_np,columns=clmns)
    pcd_pd['red'] = sigma_value.astype(np.uint8)

    #pcd_points = PyntCloud(pd.DataFrame(data=pcd_np,columns=clmns))
    pcd_points = PyntCloud(pcd_pd)
    edge_points = PyntCloud(pd.DataFrame(data=edge_np,columns=clmns))

    # pcd_points.plot()
    # edge_points.plot()

    #PyntCloud.to_file(pcd_points,output_dir+'pointcloud_edges.ply')   # Save the whole point cloud by painting the edge points
    #PyntCloud.to_file(edge_points,output_dir+'edges.ply')             # Save just the edge points
    return edge_np[:,0:3]

def PCA_alignment(Source, Target):
    """
    input: Source and Target Pointclouds as numpy.array,
            in order to work the pointcloud must have a similar shape
    output: Ann, aligned Source pointcloud, RT1, RT2, am, bm rotation and translation used
    """

    A = copy.deepcopy(Source)
    B = copy.deepcopy(Target)
    # A = np.reshape(A,(3,len(A)))
    am = np.mean(A,axis=0)
    # A-=am
    #B = np.reshape(B,(3,len(B)))
    bm = np.mean(B,axis=0)
    B-=bm
    # M = np.cov(A)
    pca=PCA(n_components=3)
    pca.fit(A)
    v1 = pca.components_[1]
    v2 = pca.components_[0]
    v3 = pca.components_[2]
    norm = np.linalg.norm(v1)
    v1= list(v1/norm)
    norm = np.linalg.norm(v2)
    v2= list(v2/norm)
    norm = np.linalg.norm(v3)
    v3= list(v3/norm)
    RT1 = np.asarray(FrankCommons.VersorsToRotMatrix(v1, v2, v3))
    print(np.linalg.det(RT1))
    if np.linalg.det(RT1)<0:
         RT1[2,:]*=-1
    An = (RT1.T@A.T).T   #rotazione inversa 

    pca1=PCA(n_components=3)
    pca1.fit(B)
    v1 = pca1.components_[0]
    v2 = pca1.components_[1]
    v3 = pca1.components_[2]
    norm = np.linalg.norm(v1)
    v1= list(v1/norm)
    norm = np.linalg.norm(v2)
    v2= list(v2/norm)
    v2_ = copy.deepcopy(v1)
    v1_ = copy.deepcopy(v2)
    norm = np.linalg.norm(v3)
    v3= list(v3/norm)
    v3_ = list(-np.asarray(v3))
    
    RT2_1 = np.asarray(FrankCommons.VersorsToRotMatrix(v1, v2, v3))
    RT2_2 = np.asarray(FrankCommons.VersorsToRotMatrix(v1_, v2_, v3_))
    print(np.linalg.det(RT2_1))
    if np.linalg.det(RT2_1)<0:
         RT2_1[2,:]*=-1
    if np.linalg.det(RT2_2)<0:
         RT2_2[2,:]*=-1
    #try the 2 rotations

    Atemp1 = (RT2_1@copy.deepcopy(An).T).T
    Atemp2 = (RT2_2@copy.deepcopy(An).T).T
    frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=500)
    # o3d.visualization.draw_geometries([NumpyToPCD(Atemp1-np.mean(Atemp1,axis=0)).paint_uniform_color([1, 0, 0]),NumpyToPCD(Atemp2-np.mean(Atemp2,axis=0)).paint_uniform_color([0, 0, 1]), NumpyToPCD(copy.deepcopy(Target)-np.mean(copy.deepcopy(Target),axis=0)), frame])
    tree = KDTree(copy.deepcopy(Target)-np.mean(copy.deepcopy(Target),axis=0))
    dist1 = tree.query(Atemp1-np.mean(Atemp1,axis=0),k=1)[0]
    dist2 = tree.query(Atemp2-np.mean(Atemp2,axis=0),k=1)[0]
    error1 =np.sqrt(np.sum(dist1**2)/len(dist1))
    error2 =np.sqrt(np.sum(dist2**2)/len(dist2))
    if error1<error2:
        RT2  = RT2_1
        o3d.visualization.draw_geometries([NumpyToPCD(Atemp1-np.mean(Atemp1,axis=0)).paint_uniform_color([0, 0, 1]), NumpyToPCD(copy.deepcopy(Target)-np.mean(copy.deepcopy(Target),axis=0)).paint_uniform_color([1, 0, 0])])
    else:
        RT2 = RT2_2
        o3d.visualization.draw_geometries([NumpyToPCD(Atemp2-np.mean(Atemp2,axis=0)).paint_uniform_color([0, 0, 1]), NumpyToPCD(copy.deepcopy(Target)-np.mean(copy.deepcopy(Target),axis=0)).paint_uniform_color([1, 0, 0])])
    
    Ann = (RT2@An.T).T
    am1 = np.mean(Ann,axis=0)
    Ann-=am1
    Ann +=bm

    # x0 = [0,0,0,0]
    # bounds = [(-180,180),(-100,100),(-100,100),(-100,100)]
    # res = minimize(error_fun,x0, args = (Ann, B),bounds = bounds)
    # Rotate by 90° to correct if 
    thet = [-90, 0 , 90]
    errors = []
    # for EZ in thet:
    #     EZ = math.radians(EZ)
    #     Source = copy.deepcopy(Ann)
    #     Target = copy.deepcopy(B)
    #     RT = [[math.cos(EZ), -math.sin(EZ), 0],
    #         [math.sin(EZ), math.cos(EZ), 0],
    #         [0, 0, 1]]
    #     Source = (RT@Source.T).T    
    #     Source -= np.mean(Source,axis = 0)
    #     Source += np.mean(Target,axis = 0)
    #     tree = KDTree(Target)
    #     dist = tree.query(Source)[0]
    #     errors.append(np.sqrt(np.sum(dist**2)/len(dist)))

    #     o3d.visualization.draw_geometries([NumpyToPCD(Source).paint_uniform_color([0, 0, 1]), NumpyToPCD(Target).paint_uniform_color([1, 0, 0])])
    print(errors)

    return Ann, RT1, RT2, am1, bm
def error_fun(x0, Source,Target):
    #rotazione attorno a Z
    EZ, OX, OY,OZ = x0
    EZ = math.radians(EZ)
    RT = [[math.cos(EZ), -math.sin(EZ), 0],
          [math.sin(EZ), math.cos(EZ), 0],
          [0, 0, 1]]

    Source = (RT@Source.T).T + [OX,OY,OZ]
    Source -= np.mean(Source,axis = 0)
    Source += np.mean(Target,axis = 0)
    tree = KDTree(Target)
    dist = tree.query(Source)[0]
    error =np.sqrt(np.sum(dist**2)/len(dist))
    o3d.visualization.draw_geometries([NumpyToPCD(Source).paint_uniform_color([0, 0, 1]), NumpyToPCD(Target).paint_uniform_color([1, 0, 0])])
    print(error)
    return error
def ICP_Registration(Source,Target,npoints = 50000,bound_l = 0, bound_d = 0,sparse = False):
        '''
        ICP registration algorithm, found optimal value to
        compute trasformation to align Source pointcloud
        to target point cloud.
        return: RT and offset to align source to target
        '''
        
        if len(Source)>npoints:
            Source = np.asarray(random.sample(list(Source), npoints))
        if len(Target)>npoints:
            Target = np.asarray(random.sample(list(Target), npoints))
        args = (Source, Target,sparse)

        x = [0,0,0,0,0,0]
        bnds = ((-bound_l,bound_l),(-bound_l,bound_l),(-bound_l,bound_l),(-bound_d,bound_d),(-bound_d,bound_d),(-bound_d,bound_d))
        if bound_l != 0 :
            res = minimize(Error_ICP_function, x, args = args,method='Powell', tol=0.001, bounds = bnds)
        else:
            res = minimize(Error_ICP_function, x, args = args,method='Powell', tol=0.001, options ={"maxiter": 100})

        [OX,OY,OZ,EX,EY,EZ] = res.x
        RT = Euler2RotMatrix([EZ,EY,EX])
        RT = np.asarray(RT)
        off = [OX,OY,OZ]
    
        return RT, np.asarray(off), float(res.fun)

def Error_ICP_function(x,Source,Target,sparse):
    """
    cost function fo ICP minimization
    """
    global count

    [OX,OY,OZ,EX,EY,EZ] = x
    print(x)
    RT = Euler2RotMatrix([EZ,EY,EX])
    RT = np.asarray(RT)

    Source_New = (RT@Source.T).T + [OX,OY,OZ]
    # if count%20 ==0:
    #     o3d.visualization.draw_geometries([NumpyToPCD(Source_New).paint_uniform_color([1, 0, 0]),NumpyToPCD(Target).paint_uniform_color([0, 0, 1])])
    tree = KDTree(Target)
    distances, index = tree.query(Source_New)
    if sparse==True:
        trsh = np.percentile(distances,95)
        bol = distances<trsh
        distances = distances[bol]
    errors = math.sqrt(sum(distances**2/len(distances))) 
    count+=1
    print(errors)
    return errors

