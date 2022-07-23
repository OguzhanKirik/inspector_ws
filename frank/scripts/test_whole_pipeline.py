import open3d as o3d
import sys
from pathlib import Path
include_dir = str(Path(__file__).resolve().parent.parent) + "/include/"
sys.path.append(include_dir)
print(include_dir)
from REGISTRATION import PC_registration
#from REGISTRATION.ICP_utils import *
from UTILS.pcd_numpy_utils import *
from STEP_ANALYSIS import STEPAnalyzer, utils_STEP
import copy



#######################################
######                           ######
###### SPARSE ALIGNMENT WORKFLOW ######
######                           ######
#######################################


step_analyzer = STEPAnalyzer.StepAnalyzer()
registration_hndl = PC_registration.PointCloudRegistration()
frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=500)

#OPEN SOURCE AND TARGET PC


#filter data
OurMockup = True
if OurMockup :  
        filename = include_dir+"/data/mockup_06_12.stp"
        alignment = True
        newAlg = False
        pcd1 = o3d.io.read_point_cloud(include_dir+"/data/PointCloudALL100k.pcd")
        pcd0 = o3d.io.read_point_cloud(include_dir+"/data/mockup_fori_colori1000k.ply")
else:
        filename = include_dir+"/data/Skullcap Hole Identification_allaxis_colors.stp"
        alignment = True
        newAlg = True
        pcd1 = o3d.io.read_point_cloud(include_dir+"/data/SparseScan_real.pcd")
        pcd0 = o3d.io.read_point_cloud(include_dir+"/data/SkullCap_pointcloud.ply")
## TEST FIND AXIS WITH COLOR
# EXTRACT MIDDLE, FIRST AND LAST POINTS OF EACH AXIS
m_ls, first_ls, last_ls = step_analyzer.find_axis_color(filename,[[1,0,0],[1,1,0]])

Source = PCDToNumpy(pcd0)
Target = PCDToNumpy(pcd1)

o3d.visualization.draw_geometries([pcd0.paint_uniform_color([1, 0, 0]),pcd1.paint_uniform_color([0, 1, 0]), NumpyToPCD(m_ls).paint_uniform_color([0,0,0])])

### filter sparse pointcloud



if alignment ==True:
        #TEST WITH MANUAL ALIGNMENT
        # #return the aligned CAD pointcloud and list of rotation and translation 
        #Source_transf, RT_ls, T_ls = registration_hndl.Manual_SparseAlignment(Source,Target)
        VOX =20
        error  =100
        while error>10:
                Source_transf, RT_ls, T_ls, error = registration_hndl.SparseAlignment(Source,Target, VOX)
                VOX = VOX/2
                if VOX<5:
                        break

        #
        # DISPLAY THE ALIGNED POINTCLOUDS
        o3d.visualization.draw_geometries([pcd0.paint_uniform_color([1, 0, 0]),pcd1.paint_uniform_color([0, 1, 0]),NumpyToPCD(Source_transf).paint_uniform_color([0, 0, 1])]) 
        # # SAVE THE ALIGNED CAD POINTCLOUD
        saveCAD=True
        if saveCAD:
                o3d.io.write_point_cloud('CAD_aligned.pcd', NumpyToPCD(Source_transf))
        # TRANSFORMATION OF AXIS BASED ON THE OBTAINED list of rotation and translation 
        m_ls =registration_hndl.Transformation_with_list(m_ls,RT_ls, T_ls)
        first_ls =registration_hndl.Transformation_with_list(first_ls,RT_ls, T_ls)   
        last_ls =registration_hndl.Transformation_with_list(last_ls,RT_ls, T_ls)

#cluster and ordering points in order to get the xy orientation of tcp based on pca of each cluster
#(forProfi option downsample the cluster in order to obtain a linear trajectory for the profilometer)
#newAlg use new clustering algorithm, works better with the skullcap_file.step
[cluster_mid, cluster_last, cluster_first, vectors, clusters_labels], [cluster_mid_profi,cluster_first_profi,cluster_last_profi, vectors, clusters_labels_profi] = step_analyzer.Cluster_and_OrderPoints(m_ls,last_ls,first_ls,forProfi=True,newAlg = newAlg)
#ROBOT POSECOMPUTATION FOR PROFILOMETER DENSE SCAN  (forProfi option return the pose offsetted from object of 500mm)

[profi_coords_ls, profi_quat_ls, clusters_labels_profi] = step_analyzer.TCP_pose_from_axis_mockup(cluster_mid_profi, cluster_last_profi, cluster_first_profi, vectors, clusters_labels_profi,
                                                                                                        filepath = 'profi_tcp_pose.txt', forProfi =True, SaveFile = True,offset = 500 )
#RETURN POSITION AND ORIENTATION OF ROBOT TCP TO PERFORM THE DENSE SCANf


#RETURN ROBOT TCP FOR DRILLING BEFORE DENSE SCAN , is just used to check the differencies
[drill_coords_ls, drill_quat_ls, clusters_labels] = step_analyzer.TCP_pose_from_axis_mockup(cluster_mid, cluster_last, cluster_first, vectors, clusters_labels,
                                                                                                        filepath = 'drill_tcp_pose.txt', forProfi =False, SaveFile = True )


#########################################à
######                            #########
########  DENSE ALIGNMENT TEST    ##########
########                          ###########
#############################################






#### each cluster position is corrected with each dense scan
new_poses = []
for i in range(max(clusters_labels)+1):

        dense0_pcd = o3d.io.read_point_cloud(include_dir+'/data/DENSE_7_12/PointCloud_'+str(i)+'.pcd')
        #instead of taking just one dense scan will take the 2 closest

        if i == 0:
                dense1_pcd = o3d.io.read_point_cloud(include_dir+'/data/DENSE_7_12//PointCloud_'+str(i+2)+'.pcd')
        else:
                dense1_pcd = o3d.io.read_point_cloud(include_dir+'/data/DENSE_7_12/PointCloud_'+str(i-1)+'.pcd')
        if i == max(clusters_labels):
              dense2_pcd = o3d.io.read_point_cloud(include_dir+'/data/DENSE_7_12/PointCloud_'+str(i-2)+'.pcd')  
        else:
             dense2_pcd = o3d.io.read_point_cloud(include_dir+'/data/DENSE_7_12/PointCloud_'+str(i+1)+'.pcd')  


        dense0 = PCDToNumpy(dense0_pcd)
        dense1 = PCDToNumpy(dense1_pcd)
        dense2 = PCDToNumpy(dense2_pcd)

        dense0 = FilterWorkingVolume(dense0)
        dense1 = FilterWorkingVolume(dense1)
        dense2 = FilterWorkingVolume(dense2)
        dense_tot = np.concatenate([dense0,dense1,dense2],axis = 0)
        TEMP_SOURCE = copy.deepcopy(Source_transf)
        TEMP_SOURCE, RT_ls, T_ls = registration_hndl.DenseAlignment(TEMP_SOURCE, dense_tot)
        temp_cluster_mid = copy.deepcopy(cluster_mid[i])
        temp_cluster_first = copy.deepcopy(cluster_first[i])
        temp_clsuter_last= copy.deepcopy(cluster_last[i])
        temp_cluster_mid =registration_hndl.Transformation_with_list(temp_cluster_mid,RT_ls, T_ls)
        temp_cluster_first =registration_hndl.Transformation_with_list(temp_cluster_first,RT_ls, T_ls)
        temp_clsuter_last =registration_hndl.Transformation_with_list(temp_clsuter_last,RT_ls, T_ls)
        label = []
        for j in range(len(clusters_labels)):
                if clusters_labels[j] ==i:
                        label.append([i]) 
        label = np.squeeze(np.asarray(label))
        #cluster_mid, cluster_last, cluster_first, vectors, clusters_labels = step_analyzer.Cluster_and_OrderPoints(temp_m_ls,temp_last_ls,first_ls,forProfi=False)
        res = step_analyzer.TCP_pose_from_axis_mockup(np.asarray([temp_cluster_mid]), np.asarray([temp_clsuter_last]), np.asarray([temp_cluster_first]), 
                                                                           np.asarray([vectors[i]]), label, filepath = "drilling_pose"+str(i)+".txt",forProfi = False, SaveFile = True )
        drill_pts_n = res[0] 
        Quat_ls = res[1]
        new_poses.append([drill_pts_n,Quat_ls])

        o3d.io.write_point_cloud('mid_dense'+str(i)+'.pcd', NumpyToPCD(temp_cluster_mid ))
        o3d.io.write_point_cloud('first_dense'+str(i)+'.pcd', NumpyToPCD(temp_cluster_first))
        o3d.io.write_point_cloud('last_dense'+str(i)+'.pcd', NumpyToPCD(temp_clsuter_last ))

#VISUALIZATION OF THE CORRECTION OF DRILLING POINTS
        draw = []
        for i in range(len(temp_cluster_mid)):
                draw.append(o3d.geometry.TriangleMesh.create_sphere(radius = 3, resolution=5).translate(drill_pts_n[i]).paint_uniform_color([1, 0, 0])) #
        for i in range(len(cluster_mid)):
                draw.append(o3d.geometry.TriangleMesh.create_sphere(radius = 3, resolution=5).translate(drill_coords_ls[i]).paint_uniform_color([0, 0, 0])) #black
        draw.append(NumpyToPCD(Source_transf).paint_uniform_color([1, 1, 0]))       
        draw.append(NumpyToPCD(TEMP_SOURCE).paint_uniform_color([0, 0, 1]))       
        draw.append(NumpyToPCD(dense_tot).paint_uniform_color([0, 1, 0]))      
        draw.append(frame)
        o3d.visualization.draw_geometries(draw)

off_pts = np.concatenate(np.asarray(new_poses)[:,0],axis=0)
Quat_ls = np.concatenate(np.asarray(new_poses)[:,1],axis=0)
g= open( "drill_tcp_pose.txt", "w")
#g.write("id\tX[mm]\tY[mm]\tZ[mm]\tq1\tq2\tq3\tq4\n")        # column names
np.savetxt(g, np.transpose([clusters_labels,off_pts[:,0],off_pts[:,1],off_pts[:,2],Quat_ls[:,0],Quat_ls[:,1],Quat_ls[:,2],Quat_ls[:,3]]))
g.close()