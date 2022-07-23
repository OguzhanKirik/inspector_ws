#! /usr/bin/env python3
import rospy
import sys
from os.path import dirname, abspath
from frank.srv import *
from frank.msg import *
import numpy as np
# include path python
include_dir = str(dirname(dirname(abspath(__file__)))) + '/include/'
sys.path.append(include_dir)
from REGISTRATION import PC_registration
from REGISTRATION import ICP_utils
#from REGISTRATION.ICP_utils import *
from UTILS.pcd_numpy_utils import *
from STEP_ANALYSIS import STEPAnalyzer, utils_STEP
import copy
import FrankAlign
import os
from scipy.spatial import distance
from sklearn.decomposition import PCA
from pykdtree.kdtree import KDTree
import random
def fill_cluster_msg(clusters,msg):
    
    for i in range(len(clusters)):
        clst = cluster()
        for j in len(i):
            pt = point()
            pt.x = j[0]
            pt.y = j[1]
            pt.z = j[2]
            clst.points.append(pt)
        
        msg.cluster.append(clst)
    return msg
def fill_vector_msg(vector, msg):
    for i in range(len(vector)):
        pt = point()
        pt.x = i[0]
        pt.y = i[1]
        pt.z = i[2]
        msg.points.append(pt)
    return msg    

def FilterDensePointcloud(points, cluster):
    radious = 200
    center_cluster = np.reshape(np.mean(cluster,axis = 0),(1,3))
    filt_points = []

    dist = np.linalg.norm(center_cluster - points, axis=1)
    bol = dist<radious
    filt_points = points[bol]
    return filt_points





def cad_extraction_and_clustering(req):
    ##### extract hole axis from the cad and perform the clusterization
    response = ExtractionAndClusteringResponse()
    PoseTargetfilepath = str(rospy.get_param("/PosePath/drill"))
    req_colors = rospy.get_param("Colors")
    # colors=  []
    # for rq in req_colors:
    #     r = rq.r
    #     g = rq.g
    #     b = rq.b
    #     colors.append([r,g,b])
    filename_step = str(rospy.get_param("/CAD_file/STEP"))
    Align.m_ls, Align.first_ls, Align.last_ls, Align.Colors = Align.step_analyzer.find_axis_color(filename_step,req_colors) #axis extraction based on colors
    try:
        [Align.cluster_mid, Align.cluster_last, Align.cluster_first, Align.vectors, Align.cluster_labels,Align.Colors], [Align.cluster_mid_profi,Align.cluster_first_profi,Align.cluster_last_profi, Align.vectors_profi, Align.cluster_labels_profi] = Align.step_analyzer.Cluster_and_OrderPoints(Align.m_ls,Align.last_ls,Align.first_ls,Align.Colors,forProfi=True,newAlg = Align.newAlg)
    except:
        response.error = "Some unexpected error occured during clusterization, try to call again the service"
        print(response.error)
        return response
    ### save on file robot target pose in CAD reference frame, before performing any alignment
    [profi_coords_ls, profi_quat_ls, clusters_labels_profi] = Align.step_analyzer.TCP_pose_from_axis_mockup(Align.cluster_mid_profi, Align.cluster_last_profi, Align.cluster_first_profi, Align.vectors, Align.cluster_labels_profi,
                                                                                                        filepath = PoseTargetfilepath +"/profi_tcp_pose0.txt", forProfi =True, SaveFile = True,offset = 600 )
    [coords_ls, quat_ls, clusters_labels] = Align.step_analyzer.TCP_pose_from_axis_mockup(Align.cluster_mid, Align.cluster_last, Align.cluster_first, Align.vectors, Align.cluster_labels,
                                                                                                    filepath = PoseTargetfilepath +"/drill_tcp_pose0.txt", forProfi =False, SaveFile = True,offset = 0 )


    ##### saving files needed for TCP_pose_from_axis function

    np.save(PoseTargetfilepath+"/cluster_mid_drill.npy", Align.cluster_mid)
    np.save(PoseTargetfilepath+"/cluster_first_drill.npy", Align.cluster_first)
    np.save(PoseTargetfilepath+"/cluster_last_drill.npy", Align.cluster_last)
    np.save(PoseTargetfilepath+ "/cluster_labels_drill.npy", Align.cluster_labels)

    np.save(PoseTargetfilepath+"/cluster_mid_profi.npy", Align.cluster_mid_profi)
    np.save(PoseTargetfilepath+"/cluster_first_profi.npy", Align.cluster_first_profi)
    np.save(PoseTargetfilepath+"/cluster_last_profi.npy", Align.cluster_last_profi)
    np.save(PoseTargetfilepath+ "/cluster_labels_profi.npy", Align.cluster_labels_profi)

    colors_list = np.concatenate(Align.Colors, axis=  0)
    Outs=baseStruct()
    OutsPlan=structArray()

    for i in range(len(clusters_labels_profi)):
        Outs=[]
        Outs=baseStruct()
        
        Outs.label=int(clusters_labels_profi[i])
        Outs.Pose.position.x=profi_coords_ls[i,0]
        Outs.Pose.position.y=profi_coords_ls[i,1]
        Outs.Pose.position.z=profi_coords_ls[i,2]
        Outs.Pose.orientation.w=profi_quat_ls[i,0]
        Outs.Pose.orientation.x=profi_quat_ls[i,1]
        Outs.Pose.orientation.y=profi_quat_ls[i,2]
        Outs.Pose.orientation.z=profi_quat_ls[i,3]

        
        OutsPlan.structList.append(Outs)

    response.points_profi=OutsPlan

    Outs1=baseStruct()
    OutsPlan1=structArray()

    for i in range(len(clusters_labels)):
        Outs1=[]
        Outs1=baseStruct()
        
        Outs1.label=int(clusters_labels[i])
        Outs1.Pose.position.x=coords_ls[i,0]
        Outs1.Pose.position.y=coords_ls[i,1]
        Outs1.Pose.position.z=coords_ls[i,2]
        Outs1.Pose.orientation.w=quat_ls[i,0]
        Outs1.Pose.orientation.x=quat_ls[i,1]
        Outs1.Pose.orientation.y=quat_ls[i,2]
        Outs1.color.r = colors_list[i,0]
        Outs1.color.g = colors_list[i,1]
        Outs1.color.b = colors_list[i,2]
        OutsPlan1.structList.append(Outs)

    response.points_drill=OutsPlan1
    response.error = "axis clusterization performed successfully"
    # response.cluster_mid_drill = fill_cluster_msg(Align.cluster_mid,response.cluster_mid_drill )
    # response.cluster_first_drill = fill_cluster_msg(Align.cluster_mid,response.cluster_mid_drill )
    # response.cluster_last_drill = fill_cluster_msg(Align.cluster_mid,response.cluster_mid_drill )
    # response.cluster_mid_profi = fill_cluster_msg(Align.cluster_mid_profi,response.cluster_mid_profi )
    # response.cluster_first_profi = fill_cluster_msg(Align.cluster_first_profi,response.cluster_first_profi )
    # response.cluster_last_profi = fill_cluster_msg(Align.cluster_last_profi,response.cluster_last_profi )
    # response.vectors = fill_vector_msg(Align.vectors,response.vectors)
    # response.cluster_labels_drill = fill_vector_msg(Align.clusters_labels,response.cluster_labels_drill)
    # response.cluster_labels_profi = fill_vector_msg(Align.clusters_labels_profi,response.cluster_labels_profi)
    return response


def sparse_align_cad(req):
    PoseTargetfilepath = str(rospy.get_param("/PosePath/drill"))
    # cluster_mid_drill = req.cluster_mid_drill
    # cluster_first_drill = req.cluster_first_drill
    # cluster_last_drill = req.cluster_last_drill
    # cluster_mid_profi = req.cluster_mid_profi
    # cluster_first_profi = req.cluster_first_profi
    # cluster_last_profi = req.cluster_last_profi
    # vectors = req.vectors
    # cluster_labels = req.cluster_labels_drilll
    # cluster_labels_profi = req.cluster_labels_profi
    response = SparseAlignmentResponse()
    try:
        cluster_mid_drill= np.load(PoseTargetfilepath+"/cluster_mid_drill.npy",allow_pickle=True)
        cluster_first_drill = np.load(PoseTargetfilepath+"/cluster_first_drill.npy",allow_pickle=True)
        cluster_last_drill =  np.load(PoseTargetfilepath+"/cluster_last_drill.npy",allow_pickle=True)
        cluster_labels_drill = np.load(PoseTargetfilepath+"/cluster_labels_drill.npy",allow_pickle=True)
        
        cluster_mid_profi = np.load(PoseTargetfilepath+"/cluster_mid_profi.npy",allow_pickle=True)
        cluster_first_profi = np.load(PoseTargetfilepath+"/cluster_first_profi.npy",allow_pickle=True)
        cluster_last_profi = np.load(PoseTargetfilepath+"/cluster_last_profi.npy",allow_pickle=True)
        cluster_labels_profi = np.load(PoseTargetfilepath+ "/cluster_labels_profi.npy",allow_pickle=True)

    except:
        response.error = "Axis extraction and clusterization not performed"
        print(response.error)
        return response

    State_pub=OperationState()
    State_pub.procedure = -2 
    State_pub.state=" Sparse Alignment Started"
    State_pub.error="No error"
    StatePub.publish(State_pub)
    
    
    
    frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=500)
    filename_step = str(rospy.get_param("/CAD_file/STEP"))
    #Align.m_ls, Align.first_ls, Align.last_ls = Align.step_analyzer.find_axis_color(filename_step,[[1,0,0],[1,1,0]])
    #print("Axis extracted from STEP file")
    filename_pc = str(rospy.get_param("/CAD_file/POINTCLOUD"))
    ### where to find sparse pointcloud
    if req.Path:
        filename_sparse = str(req.Path) + "/PointCloudALL.pcd"
        print("path 0")
    else:
        filename_sparse = str(rospy.get_param("/FilePath/sparse")) + "/PointCloudALL.pcd"
        
        print("path 1")
    print("reading sparse pointcloud from : "+ filename_sparse)
    pcd1 = o3d.io.read_point_cloud(filename_sparse)
    pcd0 = o3d.io.read_point_cloud(filename_pc)
    Align.Source = PCDToNumpy(pcd0)
    Align.Target = PCDToNumpy(pcd1)
    o3d.visualization.draw_geometries([pcd0.paint_uniform_color([1, 0, 0]),pcd1.paint_uniform_color([0, 1, 0]),
    NumpyToPCD(np.concatenate(cluster_mid_drill,axis=0)),NumpyToPCD(np.concatenate(cluster_last_drill,axis=0)),NumpyToPCD(np.concatenate(cluster_first_drill,axis = 0))])
    if req.mode == 1: #automatic
        VOX =50
        #error  =100
        # while error>10:
        Align.Source_transf, RT_ls, T_ls, error = Align.registration_hndl.SparseAlignment(copy.deepcopy(Align.Source),copy.deepcopy(Align.Target), VOX)
                # VOX = VOX/2
                # if VOX<5:
                        # breclusters_labels
    elif req.mode == 0: # manual
        Align.Source_transf, RT_ls, T_ls,error = Align.registration_hndl.Manual_SparseAlignment(Align.Source,Align.Target)

    if error<30:
        print("Alignment performed successfully!")
        #
        # DISPLAY THE ALIGNED POINTCLOUDS
        o3d.visualization.draw_geometries([pcd0.paint_uniform_color([1, 0, 0]),pcd1.paint_uniform_color([0, 1, 0]),NumpyToPCD(Align.Source_transf).paint_uniform_color([0, 0, 1])]) 
        # # SAVE THE ALIGNED CAD POINTCLOUD
        saveCAD=True
        if saveCAD:
                print(filename_sparse)
                o3d.io.write_point_cloud( str(rospy.get_param("/FilePath/sparse"))+'/CAD_aligned.pcd', NumpyToPCD(Align.Source_transf))
                if req.Path:
                    o3d.io.write_point_cloud( str(req.Path)+'/CAD_aligned.pcd', NumpyToPCD(Align.Source_transf))
        # TRANSFORMATION OF AXIS BASED ON THE OBTAINED list of rotation and translation 
        PoseTargetfilepath = str(rospy.get_param("/PosePath/drill"))
        ##### transform all of the clusters
        for i in range(len(cluster_mid_drill)):
            # transformation of all target points
            cluster_mid_drill[i] = Align.registration_hndl.Transformation_with_list(cluster_mid_drill[i],RT_ls, T_ls)
            cluster_first_drill[i] = Align.registration_hndl.Transformation_with_list(cluster_first_drill[i],RT_ls, T_ls)
            cluster_last_drill[i] = Align.registration_hndl.Transformation_with_list(cluster_last_drill[i],RT_ls, T_ls)
            cluster_mid_profi[i] = Align.registration_hndl.Transformation_with_list(cluster_mid_profi[i],RT_ls, T_ls)
            cluster_first_profi[i] = Align.registration_hndl.Transformation_with_list(cluster_first_profi[i],RT_ls, T_ls)
            cluster_last_profi[i] = Align.registration_hndl.Transformation_with_list(cluster_last_profi[i],RT_ls, T_ls)
        o3d.visualization.draw_geometries([pcd0.paint_uniform_color([1, 0, 0]),pcd1.paint_uniform_color([0, 1, 0]),
        NumpyToPCD(np.concatenate(cluster_mid_drill,axis=0)),NumpyToPCD(np.concatenate(cluster_last_drill,axis=0)),NumpyToPCD(np.concatenate(cluster_first_drill,axis = 0))])
        
        vectors = []
        #computation of new cluster vectors
        for i in range(len(cluster_mid_profi)):
            X1 = np.asarray(cluster_mid_profi[i])
            cluster_mid_profi[i] = np.asarray(cluster_mid_profi[i])
            cluster_first_profi[i] = np.asarray(cluster_first_profi[i])
            cluster_last_profi[i] = np.asarray(cluster_last_profi[i])

            pca=PCA(n_components=1)
            pca.fit(X1)
            vector = pca.components_[0]
            vectors.append(vector)
        Align.vectors = vectors
        Align.cluster_mid = cluster_mid_drill
        Align.cluster_first = cluster_first_drill
        Align.cluster_last = cluster_last_drill

        Align.cluster_mid_profi = cluster_mid_profi
        Align.cluster_first_profi = cluster_first_profi
        Align.cluster_last_profi = cluster_last_profi

        Align.cluster_labels = cluster_labels_drill
        Align.cluster_labels_profi = cluster_labels_profi


        [profi_coords_ls, profi_quat_ls, cluster_labels_profi] = Align.step_analyzer.TCP_pose_from_axis_mockup(cluster_mid_profi, cluster_last_profi, cluster_first_profi, vectors, cluster_labels_profi,
                                                                                                        filepath = PoseTargetfilepath +"/profi_tcp_pose1.txt", forProfi =True, SaveFile = True,offset = 500 )
        [coords_ls, quat_ls,cluster_labels_drill] = Align.step_analyzer.TCP_pose_from_axis_mockup(cluster_mid_drill, cluster_last_drill, cluster_first_drill, vectors, cluster_labels_drill,
                                                                                                        filepath = PoseTargetfilepath +"/drill_tcp_pose1.txt", forProfi =False, SaveFile = True,offset = 0 )
        
        
        Outs=baseStruct()
        OutsPlan=structArray()

        for i in range(len(cluster_labels_profi)):
            Outs=[]
            Outs=baseStruct()
            
            Outs.label=int(cluster_labels_profi[i])
            Outs.Pose.position.x=profi_coords_ls[i,0]
            Outs.Pose.position.y=profi_coords_ls[i,1]
            Outs.Pose.position.z=profi_coords_ls[i,2]
            Outs.Pose.orientation.w=profi_quat_ls[i,0]
            Outs.Pose.orientation.x=profi_quat_ls[i,1]
            Outs.Pose.orientation.y=profi_quat_ls[i,2]
            Outs.Pose.orientation.z=profi_quat_ls[i,3]
            OutsPlan.structList.append(Outs)
        print("target points profi")
        #print(OutsPlan)
        response.points_profi=OutsPlan
        
        Outs1=baseStruct()
        OutsPlan1=structArray()
        for i in range(len(cluster_labels_drill)):
            Outs1=[]
            Outs1=baseStruct()
            
            Outs1.label=int(cluster_labels_drill[i])
            Outs1.Pose.position.x=coords_ls[i,0]
            Outs1.Pose.position.y=coords_ls[i,1]
            Outs1.Pose.position.z=coords_ls[i,2]
            Outs1.Pose.orientation.w=quat_ls[i,0]
            Outs1.Pose.orientation.x=quat_ls[i,1]
            Outs1.Pose.orientation.y=quat_ls[i,2]
            Outs1.Pose.orientation.z=quat_ls[i,3]
            
            OutsPlan1.structList.append(Outs1)

        response.points_drill=OutsPlan1
        response.res = 1
        response.error = "sparse alignment performed successfully, pose target saved"
        print("target points drill")
        print(OutsPlan1)
        print(response.error)
        Align.SparseAlignment = True

        State_pub=OperationState()
        State_pub.procedure = -2 
        State_pub.state=" Sparse Alignment Completed"
        State_pub.error="No error"
        StatePub.publish(State_pub)
        


    else:
        response.res = 0
        
        State_pub=OperationState()
        State_pub.procedure = -2 
        State_pub.state=" Sparse Alignment Completed"
        State_pub.error="Unable to perform the alignment, go for manual mode or acquire new PC"
        StatePub.publish(State_pub)
        
        
        response.error = "it was unable to perform alignment, try changing alignment mode!" 
        print(response.error)
    return response


def dense_align_cad(req):
    #location of dense pointcloud
    if req.Path:
        filepath_dense = str(req.Path)
        print("path 0")
    else:
        filepath_dense = str(rospy.get_param("/FilePath/dense"))
        
        print("path 1")
    print("reading dense pointcloud from: " + filepath_dense)
    response = DenseAlignmentResponse()
    if Align.SparseAlignment ==False:
        response.error = "Sparse Alignment not performed, please perform it!"
        response.res  = 0
        return response
    else:
        new_pts = []
        new_quats= []
        n = req.cluster_id

        State_pub=OperationState()
        State_pub.procedure = -2 
        State_pub.state=" Dense Alignment Started"
        State_pub.error="No error"
        StatePub.publish(State_pub)
        

        ### build the list of all available dense scan
        # num_ls = []
        # for i in range(1000):
        #     if os.path.isfile(dense_path+'/PointCloud_'+str(i)+'.pcd'):
        #         num_ls.append(i)
        # print("cluster acquired are : " , num_ls)
        # for i in range(len(num_ls)):#Align.clusters_labels)+1):
        #         n = num_ls[i]
        if  Align.DensePointcloud ==[]:
            #check if is already saved DensePointCloudALL
            if os.path.isfile(filepath_dense+'/DensePointCloudALL.pcd'):
                print("loading this dense pointcloud all:  " + str(filepath_dense+'/DensePointCloudALL.pcd'))
                Align.DensePointcloud = PCDToNumpy(o3d.io.read_point_cloud(filepath_dense+'/DensePointCloudALL.pcd'))
            else:
                print("combining all pointcloud")
                    ## build the list of all available dense scan
                num_ls = []
                for i in range(2000):
                    if os.path.isfile(filepath_dense+'/PointCloud_'+str(i)+'.pcd'):
                        num_ls.append(i)
                print(num_ls)
                all_pointcloud = []
                for i in num_ls:
                    print(i)
                    dense0_pcd = o3d.io.read_point_cloud(filepath_dense+'/PointCloud_'+str(i)+'.pcd')
                    dense0 = PCDToNumpy(dense0_pcd)
                    dense0 = FilterWorkingVolume(dense0)
                    all_pointcloud.append(dense0)
                    
                Align.DensePointcloud = np.concatenate(all_pointcloud,axis=  0)
                #downsample random to 2 milion points
                #Align.DensePointcloud = np.asarray(random.sample(list(Align.DensePointcloud), 10000000))
                o3d.io.write_point_cloud(filepath_dense+'/DensePointCloudALL.pcd',NumpyToPCD(Align.DensePointcloud))
                #o3d.io.write_point_cloud(filepath_dense+'/DensePointCloudALL.ply',NumpyToPCD(Align.DensePointcloud))
       # o3d.visualization.draw_geometries([NumpyToPCD(Align.DensePointcloud)])
        
        print("Starting alignment dense scan : ",n)
        #instead of taking just one dense scan will take the 2 closest

        # if i == 0:
        #         n1 = num_ls[i+2]
        #         dense1_pcd = o3d.io.read_point_cloud(dense_path+'/PointCloud_'+str(n1)+'.pcd')
        # else:
        #         n1 = num_ls[i-1]
        #         dense1_pcd = o3d.io.read_point_cloud(dense_path+'/PointCloud_'+str(n1)+'.pcd')
        # if i == len(num_ls)-1:
        #     n2 = num_ls[i-2]
        #     dense2_pcd = o3d.io.read_point_cloud(dense_path+'/PointCloud_'+str(n2)+'.pcd')  
        # else:
        #     n2 = num_ls[i+1]
        #     dense2_pcd = o3d.io.read_point_cloud(dense_path+'/PointCloud_'+str(i+1)+'.pcd')  


        # #dense0 = PCDToNumpy(dense0_pcd)
        # # dense1 = PCDToNumpy(dense1_pcd)
        # # dense2 = PCDToNumpy(dense2_pcd)

        # dense0 = FilterWorkingVolume(dense0)
        # dense1 = FilterWorkingVolume(dense1)
        # dense2 = FilterWorkingVolume(dense2)
        TEMP_SOURCE = copy.deepcopy(Align.Source_transf)
        temp_cluster_mid = copy.deepcopy(Align.cluster_mid[n])
        temp_cluster_first = copy.deepcopy(Align.cluster_first[n])
        temp_cluster_last= copy.deepcopy(Align.cluster_last[n])
        dense_tot = FilterDensePointcloud(Align.DensePointcloud, temp_cluster_mid) # np.concatenate([dense0,dense1,dense2],axis = 0)
        # index = np.random.choice(dense_tot.shape[0],500000, replace = False)
        # dense_tot =dense_tot[index]
        # dense_tot = ICP_utils.Edge_Detection(dense_tot, 100, 0.02)

        o3d.visualization.draw_geometries([NumpyToPCD(TEMP_SOURCE).paint_uniform_color([1,0,0]),NumpyToPCD(dense_tot).paint_uniform_color([0,0,1]),NumpyToPCD(temp_cluster_first), NumpyToPCD(temp_cluster_last)])

        TEMP_SOURCE, RT_ls, T_ls = Align.registration_hndl.DenseAlignment(TEMP_SOURCE, dense_tot)

        temp_cluster_mid =Align.registration_hndl.Transformation_with_list(temp_cluster_mid,RT_ls, T_ls)
        temp_cluster_first =Align.registration_hndl.Transformation_with_list(temp_cluster_first,RT_ls, T_ls)
        temp_cluster_last =Align.registration_hndl.Transformation_with_list(temp_cluster_last,RT_ls, T_ls)
        label = []
        for j in range(len(Align.cluster_labels)):
                if Align.cluster_labels[j] ==n:
                        label.append([n]) 
        label = np.squeeze(np.asarray(label))
        #cluster_mid, cluster_last, cluster_first, vectors, clusters_labels = step_analyzer.Cluster_and_OrderPoints(temp_m_ls,temp_last_ls,first_ls,forProfi=False)
        res = Align.step_analyzer.TCP_pose_from_axis_mockup(np.asarray([temp_cluster_mid]), np.asarray([temp_cluster_last]), np.asarray([temp_cluster_first]), 
                                                                        np.asarray([Align.vectors[n]]), label, filepath =  str(rospy.get_param("/PosePath/drill")) +"/drill_tcp_pose_CLUSTER"+str(n)+".txt",forProfi = False, SaveFile = True )
        drill_pts_n =(res[0])
        Quat_ls = res[1]
        o3d.visualization.draw_geometries([NumpyToPCD(TEMP_SOURCE).paint_uniform_color([1,0,0]),NumpyToPCD(dense_tot).paint_uniform_color([0,0,1]),NumpyToPCD(temp_cluster_first), NumpyToPCD(temp_cluster_last)  ])
        # new_pts.append(drill_pts_n)
        # new_quats.append(Quat_ls)
        print("Finished alignment dense scan : ",n)
                # o3d.io.write_point_cloud('mid_dense'+str(i)+'.pcd', NumpyToPCD(temp_cluster_mid ))
                # o3d.io.write_point_cloud('first_dense'+str(i)+'.pcd', NumpyToPCD(temp_cluster_first))
                # o3d.io.write_point_cloud('last_dense'+str(i)+'.pcd', NumpyToPCD(temp_clsuter_last ))

        # #VISUALIZATION OF THE CORRECTION OF DRILLING POINTS
        #         draw = []
        #         for i in range(len(temp_cluster_mid)):
        #                 draw.append(o3d.geometry.TriangleMesh.create_sphere(radius = 3, resolution=5).translate(drill_pts_n[i]).paint_uniform_color([1, 0, 0])) #
        #         for i in range(len(Align.cluster_mid)):
        #                 draw.append(o3d.geometry.TriangleMesh.create_sphere(radius = 3, resolution=5).translate(drill_coords_ls[i]).paint_uniform_color([0, 0, 0])) #black
        #         draw.append(NumpyToPCD(Align.Source_transf).paint_uniform_color([1, 1, 0]))       
        #         draw.append(NumpyToPCD(TEMP_SOURCE).paint_uniform_color([0, 0, 1]))       
        #         draw.append(NumpyToPCD(dense_tot).paint_uniform_color([0, 1, 0]))      
        #         #draw.append(frame)
        #         o3d.visualization.draw_geometries(draw)

        # off_pts = np.concatenate(np.asarray(new_pts),axis=0)
        # Quat_ls = np.concatenate(np.asarray(new_quats),axis=0)
        # PoseTargetfilepath = str(rospy.get_param("/PosePath/drill")) +"/drill_tcp_pose.txt"
        # g= open( PoseTargetfilepath, "w")
        # #g.write("id\tX[mm]\tY[mm]\tZ[mm]\tq1\tq2\tq3\tq4\n")        # column names
        # np.savetxt(g, np.transpose([Align.clusters_labels,off_pts[:,0],off_pts[:,1],off_pts[:,2],Quat_ls[:,0],Quat_ls[:,1],Quat_ls[:,2],Quat_ls[:,3]]))
        # g.close()

        Outd=baseStruct()
        OutdPlan=structArray()

        for i in range(len(list(label))):
            Outd=[]
            Outd=baseStruct()
            
            Outd.label=int(label[i])
            Outd.Pose.position.x=drill_pts_n[i,0]
            Outd.Pose.position.y=drill_pts_n[i,1]
            Outd.Pose.position.z=drill_pts_n[i,2]
            Outd.Pose.orientation.w=Quat_ls[i,0]
            Outd.Pose.orientation.x=Quat_ls[i,1]
            Outd.Pose.orientation.y=Quat_ls[i,2]
            Outd.Pose.orientation.z=Quat_ls[i,3]
            
            OutdPlan.structList.append(Outd)
        
        
        State_pub=OperationState()
        State_pub.procedure = -2 
        State_pub.state=" Dense Alignment Completed"
        State_pub.error="No error"
        StatePub.publish(State_pub)
        
        response.points=OutdPlan
        response.res = 1
        return response



    
if __name__ == '__main__':

    try:
        rospy.init_node('step_align_node')
        StatePub=rospy.Publisher("/operation_state", OperationState, queue_size=1)
        #trigger_sub = rospy.Subscriber("trigger_State", Int32, trigger_callback)
        Align=  FrankAlign.newAlign()
        Align.step_analyzer = STEPAnalyzer.StepAnalyzer()
        Align.registration_hndl = PC_registration.PointCloudRegistration()
        s_sparse = rospy.Service("sparse_alignment", SparseAlignment, sparse_align_cad)
        s_dense = rospy.Service("dense_alignment", DenseAlignment, dense_align_cad)
        s_cad_clust = rospy.Service("axis_clustering",ExtractionAndClustering, cad_extraction_and_clustering )
        print("alignment is ready to go!")

        rospy.spin()
    except  rospy.ROSInterruptException:
        pass


