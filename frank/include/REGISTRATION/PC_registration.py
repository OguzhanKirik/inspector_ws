import open3d as o3d
import sys
from pathlib import Path
from os.path import dirname, abspath
# include path python
include_dir = str(Path(__file__).resolve().parent)
sys.path.append(include_dir)
print(include_dir)
include_dir = str(Path(__file__).resolve().parent.parent)
sys.path.append(include_dir)
print(include_dir)
include_dir = str(Path(__file__).resolve().parent.parent.parent)
sys.path.append(include_dir)
print(include_dir)
import teaserpp_python
import numpy as np 
import copy
from helpers import pcd2xyz, extract_fpfh, get_teaser_solver, Rt2T, find_correspondences
import ICP_utils
from UTILS.pcd_numpy_utils import *
from scipy.spatial.transform import Rotation as Rot
import cv2
from pykdtree.kdtree import KDTree

# 
#
#
#

class PointCloudRegistration():
    def DenseAlignment(self,Source,Target):
        '''
        input: CAD pointcloud (Source) and Local dense scan of the workpiece (Target) as numpy array
        output: Transformed Source, rotation list RT_ls and translation list T_ls
        '''
        print('beginning dense alignment')
        ##
        ## ICP registration
        ##
        RT_TOT = []
        T_TOT = []
        #invert source and target since its easier to align the piece with the total
        # frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=500)
        # o3d.visualization.draw_geometries([NumpyToPCD(Source).paint_uniform_color([1, 0, 0]),NumpyToPCD(Target).paint_uniform_color([0, 1, 0]), frame]) # used just to show the initial position
        temp_source = copy.deepcopy(Target)
        temp_target = copy.deepcopy(Source)
        RT, O, error = ICP_utils.ICP_Registration(temp_source,temp_target,npoints = 50000, bound_l = 10, bound_d  = 0.1 )

        Source_n = copy.deepcopy((RT.T@Source.T).T - O)
        # o3d.visualization.draw_geometries([NumpyToPCD(Source_n).paint_uniform_color([0, 0, 0]),NumpyToPCD(Source).paint_uniform_color([1, 0, 0]),
        #                         NumpyToPCD(Target).paint_uniform_color([0, 1, 0]), frame])          # used just to show the final position

        RT_TOT.append(RT.T)
        T_TOT.append(-O)

        return Source_n, RT_TOT, T_TOT


    def SparseAlignment(self,Source, Target,VOX):
        '''
        input: Source and Target pointcloud as numpy array, 
                alignment is performed stepwise, first the principal direction are aligned,
                then the footprint in teh XY plane is aligned
                then an icp registration of the keypoints is performed
                then a final icp registration of the whole pointclouds downsampled randomly
        output: Transformed Source pointcloud and ordered list of RT (rotation) and T (traslation) 
        '''
        VOXEL_SIZE =VOX  #Main parameter that affect the footprint alignment, need to be choosen wisely
        RT_TOT = []
        T_TOT = []
        VISUALIZE = False
        if VISUALIZE:
            o3d.visualization.draw_geometries([NumpyToPCD(Source).paint_uniform_color([0, 0, 1]),NumpyToPCD(Target).paint_uniform_color([1.0, 0, 0.0])])
        ########################################################
        ######## ADDED RANDOM ROTATION TO TEST + NOISE #########
        ########################################################
        #____________________________________________________________________________________________________________________________
        # mu = 0
        # sigma = 0
        # noise0 = np.random.normal(mu,sigma, size = (len(Source),3))
        # Source = Source+noise0
        # ROT = np.asarray(Rot.random().as_matrix())              # random rotation to test consistency
        # Source = (ROT@Source.T).T
        # frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=500)
        # RT_TOT.append(ROT)
        # T_TOT.append(np.asarray([0,0,0]))
        # # if VISUALIZE:
        # #     o3d.visualization.draw_geometries([NumpyToPCD(Source),NumpyToPCD(Target), frame])
        # # #____________________________________________________________________________________________________________________________

        #####################################
        ###### MEAN-SHIFT TO TARGET #########
        #####################################

        source_pmedio = np.mean(Source,axis=0)
        target_pmedio = np.mean(Target,axis=0)
        Source = Source - source_pmedio + target_pmedio
        I = np.asarray([[1,0,0],
                        [0,1,0],
                        [0,0,1]])
        T_TOT.append(-source_pmedio)
        RT_TOT.append(I)
        T_TOT.append(target_pmedio)
        RT_TOT.append(I)

        # Source_pcd = NumpyToPCD(Source)
        # Target_pcd = NumpyToPCD(Target)

        if VISUALIZE:
            o3d.visualization.draw_geometries([NumpyToPCD(Source).paint_uniform_color([0, 0, 1]),NumpyToPCD(Target).paint_uniform_color([1.0, 0, 0.0])])
        #disegno origine
        #____________________________________________________________________________________________________________________________
        ###########################
        ##### PCA ALIGNMENT #######
        ###########################
        Source_a,  RT1, RT2, am, bm=ICP_utils.PCA_alignment(copy.deepcopy(Source), copy.deepcopy(Target))

        RT_TOT.append(RT1.T)
        T_TOT.append(np.asarray([0,0,0]))
        RT_TOT.append(RT2)
        T_TOT.append(np.asarray([0,0,0]))
        T_TOT.append(-am)
        RT_TOT.append(I)
        T_TOT.append(bm)
        RT_TOT.append(I)

        Source_nn = (RT1.T@Source.T).T
        Source_nn = (RT2@Source_nn.T).T
        Source_nn = Source_nn -am
        Source_nn = Source_nn +bm

        if VISUALIZE:
            o3d.visualization.draw_geometries([NumpyToPCD(Source_nn).paint_uniform_color([0, 0, 1]),NumpyToPCD(Target).paint_uniform_color([1.0, 0, 0.0])])

        Source_nn_pcd = copy.deepcopy(NumpyToPCD(Source_nn))
        # if VISUALIZE:
        #     o3d.visualization.draw_geometries([Target_pcd.paint_uniform_color([1.0, 0, 0.0]),NumpyToPCD(Source_nn).paint_uniform_color([0, 0, 1])])
        #____________________________________________________________________________________________________________________________
        ######################################
        ##### TEASER_PP REGISTRATION #########          feature recognition alignement based on teaser++ package, used for footstamp alignment in xy plane
        ######################################
        error = 1000
        print(len(np.squeeze(Target)))
        print("error limit is : " , len(np.squeeze(Target))/1000 )
        while error>len(np.squeeze(Target))/1000:
            ##Source_nn_pcd_tmp =   o3d.geometry.keypoint.compute_iss_keypoints(Source_nn_pcd)
            #arget_pcd_tmp =  o3d.geometry.keypoint.compute_iss_keypoints(Target_pcd)
            Source_2d = copy.deepcopy(Source_nn)
            #Source_2d[:,2] = np.zeros(len(Source_2d))
            Target_2d = copy.deepcopy(Target)
            #Target_2d[:,2] = np.zeros(len(Target_2d))
            #o3d.visualization.draw_geometries([NumpyToPCD(Source_2d).paint_uniform_color([1,0,0]),NumpyToPCD(Target_2d).paint_uniform_color([0,0,1])])
            A_pcd_raw = NumpyToPCD(Source_2d)
            B_pcd_raw = NumpyToPCD(Target_2d)
            A_pcd = A_pcd_raw.voxel_down_sample(voxel_size=VOXEL_SIZE)
            B_pcd = B_pcd_raw.voxel_down_sample(voxel_size=VOXEL_SIZE)

            A_xyz = pcd2xyz(A_pcd) # np array of size 3 by N
            B_xyz = pcd2xyz(B_pcd) # np array of size 3 by M

            # extract FPFH features
            A_feats = extract_fpfh(A_pcd,VOXEL_SIZE)
            B_feats = extract_fpfh(B_pcd,VOXEL_SIZE)

            # establish correspondences by nearest neighbour search in feature space
            corrs_A, corrs_B = find_correspondences(
                A_feats, B_feats, mutual_filter=True)
            A_corr = A_xyz[:,corrs_A] # np array of size 3 by num_corrs
            B_corr = B_xyz[:,corrs_B] # np array of size 3 by num_corrs

            num_corrs = A_corr.shape[1]
            print(f'FPFH generates {num_corrs} putative correspondences.')

            # visualize the point clouds together with feature correspondences
            points = np.concatenate((A_corr.T,B_corr.T),axis=0)
            lines = []
            for i in range(num_corrs):
                lines.append([i,i+num_corrs])
            colors = [[0, 1, 0] for i in range(len(lines))] # lines are shown in green
            line_set = o3d.geometry.LineSet(
                points=o3d.utility.Vector3dVector(points),
                lines=o3d.utility.Vector2iVector(lines),
            )
            line_set.colors = o3d.utility.Vector3dVector(colors)
            if VISUALIZE:
                o3d.visualization.draw_geometries([A_pcd,B_pcd,line_set])

            # robust global registration using TEASER++
            NOISE_BOUND = VOXEL_SIZE
            teaser_solver = get_teaser_solver(NOISE_BOUND)
            teaser_solver.solve(A_corr,B_corr)
            solution = teaser_solver.getSolution()
            R_teaser = solution.rotation
            t_teaser = solution.translation
            T_teaser = Rt2T(R_teaser,t_teaser)

            # Visualize the registration results
            #A_pcd_T_teaser = copy.deepcopy(Source_nn_pcd).transform(T_teaser)
            temp_a = A_pcd.transform(T_teaser)
            temp_b = B_pcd


            # visualize the registration after ICP refinement
            Source_nn_pcd1 = copy.deepcopy(NumpyToPCD(Source_2d).transform(T_teaser))
            #Target_pcd = B_pcd
            tree = KDTree(PCDToNumpy(temp_a))
            dist = tree.query(PCDToNumpy(temp_b),k=1)[0]
            error =np.sqrt(np.sum(dist**2)/len(dist))
            print("error is : ", error)
            #o3d.visualization.draw_geometries([temp_a.paint_uniform_color([1,0,0]),temp_b.paint_uniform_color([0,0,1])])
            
           # o3d.visualization.draw_geometries([Source_nn_pcd1.paint_uniform_color([1,0,0]),NumpyToPCD(Target).paint_uniform_color([0,0,1])])
            VOXEL_SIZE = VOXEL_SIZE-1
            if VOXEL_SIZE<5:
                break

        RT_TOT.append(R_teaser) #maybe convert to numpy
        T_TOT.append(np.asarray(t_teaser))
        Source_nn = (R_teaser@Source_nn.T).T + np.asarray(t_teaser)
        #____________________________________________________________________________________________________________________________
        #
        #source and target are  now sufficiently close to use ICP
        #
        #now I invert source and target
        #temp_source_pcd = copy.deepcopy(Target_pcd)
        temp_source = copy.deepcopy(Target)
        #temp_target_pcd = copy.deepcopy(Source_nn_pcd)
        temp_target = copy.deepcopy(Source_nn)

        #######################################
        ####### ICP with Keypoints ############
        #######################################

        # keypoints0_pcd = o3d.geometry.keypoint.compute_iss_keypoints(temp_source_pcd)
        # keypoints1_pcd = o3d.geometry.keypoint.compute_iss_keypoints(temp_target_pcd)
        # keypoints0_pcd = copy.deepcopy(temp_source_pcd)
        # keypoints1_pcd = copy.deepcopy(temp_target_pcd)

        # if VISUALIZE:
        #     o3d.visualization.draw_geometries([keypoints0_pcd.paint_uniform_color([0, 0, 1]),keypoints1_pcd.paint_uniform_color([1.0, 0, 0.0])])
        # keypoints0 = PCDToNumpy(keypoints0_pcd)
        # keypoints1 = PCDToNumpy(keypoints1_pcd)
        RT3, O, error = ICP_utils.ICP_Registration(temp_source,temp_target,1000, sparse=True)
        temp_source_n = copy.deepcopy((RT3@temp_source.T).T + O)
        Source_nnn = copy.deepcopy((RT3.T@Source_nn.T).T - O) #use inverse rotation and offset

        RT_TOT.append(RT3.T)
        T_TOT.append(-O)

      

        if VISUALIZE:
            o3d.visualization.draw_geometries([NumpyToPCD(Source_nnn).paint_uniform_color([0, 0, 1]) ,NumpyToPCD(Target).paint_uniform_color([1, 0, 0]), NumpyToPCD(Source_nn).paint_uniform_color([0,1,1])])  #plot
        # # local refinement using ICP
        
        #########################################
        ######## final ICP Registration #########
        #########################################

        #now I invert again source and target
        #temp_source_pcd2 = copy.deepcopy(Target_pcd)
        temp_source2 = copy.deepcopy(Target)
        #temp_target_pcd2 = copy.deepcopy(Source_nnn_pcd)
        temp_target2 = copy.deepcopy(Source_nnn)

        RT4, O2,error = ICP_utils.ICP_Registration(temp_source2,temp_target2,npoints = 50000, sparse=True)
        temp_source_nn = copy.deepcopy((RT4@temp_source2.T).T+O2) 
        Source_nnnn = copy.deepcopy((RT4.T@Source_nnn.T).T - O2)


        RT_TOT.append(RT4.T)
        T_TOT.append(-O2)


        if VISUALIZE:
            o3d.visualization.draw_geometries([NumpyToPCD(Source_nnnn).paint_uniform_color([0, 0, 1]),NumpyToPCD(Target).paint_uniform_color([1, 0, 0]),NumpyToPCD(Source_nnn).paint_uniform_color([0, 1, 1])])
        return Source_nnnn, RT_TOT, T_TOT, error
    
    

    def Manual_SparseAlignment(self,Source,Target):
        '''
        input: Source and Target PointClouds as numpy arrays,
                need to display the pointcloud in order to get user point input
        output: Source transformed and RT , T transformation
        '''   
        Source_pcd = NumpyToPCD(Source)
        Target_pcd = NumpyToPCD(Target)
        picked_id_source = self.pick_points(Source_pcd)
        picked_id_target = self.pick_points(Target_pcd)
        RT_TOT = []
        T_TOT = []
        assert (len(picked_id_source) >= 3 and len(picked_id_target) >= 3)
        assert (len(picked_id_source) == len(picked_id_target))
        corr = np.zeros((len(picked_id_source), 2))
        corr[:, 0] = picked_id_source
        corr[:, 1] = picked_id_target
        
        # estimate rough transformation using correspondences
        print("Compute a rough transform using the correspondences given by user")
        p2p = o3d.pipelines.registration.TransformationEstimationPointToPoint()
        trans_init = p2p.compute_transformation(Source_pcd,Target_pcd,
                                                o3d.utility.Vector2iVector(corr))
        # point-to-point ICP for refinement
        print("Perform point-to-point ICP refinement")
        threshold = 0.03  # 3cm distance threshold
        reg_p2p = o3d.pipelines.registration.registration_icp(
            Source_pcd, Target_pcd, threshold, trans_init,
            o3d.pipelines.registration.TransformationEstimationPointToPoint())
        self.draw_registration_result(NumpyToPCD(Source), NumpyToPCD(Target), reg_p2p.transformation)
        print("")
        transf = reg_p2p.transformation
        RT = [transf[0][0:3],
             transf[1][0:3],
             transf[2][0:3]]
        T = np.asarray([transf[0][3], transf[1][3], transf[2][3]])
        RT_TOT.append(RT)
        T_TOT.append(T)
        Source_pcd.transform(transf)
        Source_n = PCDToNumpy(Source_pcd)
         #now I invert source and target, easier to align sparse scan to cad
        temp_source_pcd = copy.deepcopy(Target_pcd)
        temp_source = PCDToNumpy(temp_source_pcd)
        temp_target_pcd = copy.deepcopy(Source_pcd)
        temp_target = PCDToNumpy(temp_target_pcd)
        RT3, O2,error = ICP_utils.ICP_Registration(temp_source,temp_target,npoints=30000)
        temp_source_n = copy.deepcopy((RT3@temp_source.T).T+O2) 
        Source_nn = copy.deepcopy((RT3.T@Source_n.T).T - O2) #inverse rotation and translation
        frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=500)
        o3d.visualization.draw_geometries([NumpyToPCD(Source_nn).paint_uniform_color([1, 0, 0]), Target_pcd.paint_uniform_color([0, 1, 0]), frame])

        RT_TOT.append(RT3.T)
        T_TOT.append(-O2)


        return Source_nn, RT_TOT, T_TOT, error

    def pick_points(self,pcd):
        print("")
        print(
            "1) Please pick at least three correspondences using [shift + left click]"
        )
        print("   Press [shift + right click] to undo point picking")
        print("2) After picking points, press 'Q' to close the window")
        vis = o3d.visualization.VisualizerWithEditing()
        vis.create_window()
        vis.add_geometry(pcd)
        vis.run()  # user picks points
        vis.destroy_window()
        print("")
        return vis.get_picked_points()

    def Transformation_with_list(self,Points,RT_ls, T_ls):
        """
        input: Points, np.array(N,3) that need to be transformed, RT_ls rotation matrix list, T_ls translation vector list, need to be of the same length
        output: rototranslated points list
        """
        temp = copy.deepcopy(Points)
        for i in range(len(RT_ls)):
            RT_i = RT_ls[i]
            T_i = T_ls[i]
            temp = (RT_i@temp.T).T+T_i    
        return temp

    def draw_registration_result(self,source, target, transformation):
        source_temp = copy.deepcopy(source)
        target_temp = copy.deepcopy(target)
        source_temp.paint_uniform_color([1, 0.706, 0])
        target_temp.paint_uniform_color([0, 0.651, 0.929])
        source_temp.transform(transformation)
        frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=500)
        o3d.visualization.draw_geometries([source_temp, target_temp, frame])
