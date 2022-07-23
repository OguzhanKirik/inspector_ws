import numpy as np
import sys
from os.path import dirname, abspath
import open3d as o3d
import copy

from numpy.core.function_base import linspace

include_dir = str(dirname(dirname(abspath(__file__)))) + '/include/'
sys.path.append(include_dir)
import FrankProfilometer 
from bisect import bisect_left
from frank.msg import OperationState
from UTILS.pcd_numpy_utils import *
from pykdtree.kdtree import KDTree
from scipy.optimize import minimize_scalar, minimize
import math
from UTILS import FrankUtilities
import DataSync
global counting
counting =0
global index1
index1 = 0
global index2
index2 = 0

# class data_sync:
# 	XX0 = 0
# 	YY0 = 0 
# 	ZZ0 = 0
# 	XX1 = 0
# 	YY1 = 0 
# 	ZZ1 = 0
# 	poses0 = 0
# 	timestamp0 = 0
# 	poses1 = 0
# 	timestamp1 = 0

	


def take_closest(myList, myNumber):
	"""
	Assumes myList is sorted. Returns closest value to myNumber.

	If two numbers are equally close, return the smallest number.
	"""
	pos = bisect_left(myList, myNumber)
	if pos == 0:
		return myList[0],pos
	if pos == len(myList):
		return myList[-2], pos-1
	before = myList[pos - 1]
	after = myList[pos]
	if after - myNumber < myNumber - before:
		return after, pos
	else:
		return before, pos
	#return before, after ,pos-1, pos
	
def interp_pose(before,after,time):
	"""
	interpolate robot poses basedon closest timestamp, before and after triggered profile 
	"""

	mx  = (after[0]-before[0]) / (after[7]-before[7])
	qx = -mx*before[7] + before[0] 
	x = mx*time+qx

	my  = (after[1]-before[1]) / (after[7]-before[7])
	qy = -my*before[7] + before[1] 
	y = my*time+qy

	mz  = (after[2]-before[2]) / (after[7]-before[7])
	qz = -mz*before[7] + before[2] 
	z = mz*time+qz
	pose = [x, y, z]

	mq1  = (after[3]-before[3]) / (after[7]-before[7])
	qq1 = -mq1*before[7] + before[3] 
	q1 = mq1*time+qq1

	mq2  = (after[4]-before[4]) / (after[7]-before[7])
	qq2 = -mq2*before[7] + before[4] 
	q2 = mq2*time+qq2

	mq3  = (after[5]-before[5]) / (after[7]-before[7])
	qq3 = -mq3*before[7] + before[5] 
	q3 = mq3*time+qq3

	mq4  = (after[6]-before[6]) / (after[7]-before[7])
	qq4 = -mq4*before[7] + before[6] 
	q4 = mq4*time+qq4

	pose = [x, y, z, q1, q2,q3, q4]

	return pose

def sync_function(filepath,statepub, num, DELTA_T, profi):
	"""
	sincronize poses and profiles with linear regression from the timestamp of first_buffer and response from PLC
	"""
	## parameters fro linear regression ##
	# q0 = 0.00197044634192798
	# q1 = 1.07218517612807
	# q2 = 0.07989322191284130
	# q3 = -0.8353101960425250



	print("STARTING POINTCLOUD SYNC")
	print(DELTA_T)
	#profi = FrankProfilometer.newProfilometer()
	#DELTA_T = 0.028
	#open profilometer data
	#for i in range(num):
	[XX,YY,ZZ] = profi.ReadPointCloudInputTextFile(filepath+"/PointCloud_"+str(num)+".txt")
	poses = np.loadtxt(filepath + "/egm_poses"+str(num)+".txt")
	timestamp = np.loadtxt(filepath+"/trigger_stamp"+str(num)+".txt")
	# t_req = timestamp[1,0]
	# t_resp = timestamp[2,0]
	t_buff = timestamp[0]
	initial = t_buff +DELTA_T# + q0+q1*t_resp+q2*t_buff+q3*(t_resp*t_buff)
	# initial = np.asarray(timestamp[1,0])
	prof_time = []
	for k in range(len(ZZ)):
		prof_time.append(initial+0.01*k)
	###
	## seleect poses close to profilometer timestamp
	###
	prof_time = np.asarray(prof_time)
	res = []
	for j in range(len(prof_time)):
		time = prof_time[j]
		res.append(take_closest(poses[:,7],time))
	index = list(np.asarray(res)[:,1].astype(int))
	#index_after = list(np.asarray(res)[:,3].astype(int))

	# poses_before=poses[index_before]
	# poses_after=poses[index_after]
	poses = poses[index]
	# for i in range(len(prof_time)):
	# 	poses.append(interp_pose(poses_before[i], poses_after[i], prof_time[i]))
	#poses = poses[:,0:7]
	poses = np.asarray(poses)

	Num = np.linspace(0,len(poses),len(poses)+1).astype(int)
	Coord = list(poses[:,0:3])
	Quat_w = list(poses[:,3])
	Quat_x = list(poses[:,4])
	Quat_y = list(poses[:,5])
	Quat_z = list(poses[:,6])

	Quat = list(np.asarray([Quat_w, Quat_x, Quat_y, Quat_z]).T)
	start = 0
	for l in range(len(Coord)):
		 #TO BE REMOVED
		if start==0:
			fCoord =  open(filepath+"/Coords"+str(num)+".txt", "w")
			fCoord.write("Number\tX [mm]\tY [mm]\tZ [mm]\tqw\tqx\tqy\tqz\n") #TO BE REMOVED
			fCoord.close()
			start=1
	
		fCoord = open(filepath+"/Coords"+str(num)+".txt", "a")
		fCoord.write(str(Num[l]) + "\t" + str(Coord[l][0]) + "\t" + str(Coord[l][1]) + "\t"
								+ str(Coord[l][2]) + "\t" + str(Quat[l][0]) + "\t" + str(Quat[l][1]) + "\t"
								+ str(Quat[l][2]) + "\t" + str(Quat[l][3]) + "\n") #TO BE REMOVED
	fCoord.close()
	start = 0                
	points = profi.BuildPointCloud_RobotCoordinate(Num, Coord, Quat, XX, YY, ZZ, SavePath=filepath +"/PointCloud_"+str(num)+".pcd")
	print("pointcloud sincronized")
	#statepub = rospy.Publisher("/operation_state", OperationState, queue_size =1)
	if num>=0:
		msg = OperationState()
		msg.procedure = 0
		msg.state = "cluster saved :  " + str(num)
		msg.error = "No error"
		statepub.publish(msg)
	return points
#sync_function("/home/c301/Desktop/TEMP/data_3_12",3)
def sync_function_calib(DELTA_T,filepath, nums,data):
	"""
	sincronize poses and profiles with linear regression from the timestamp of first_buffer and response from PLC
	"""
	## parameters fro linear regression ##
	# q0 = 0.00197044634192798
	# q1 = 1.07218517612807
	# q2 = 0.07989322191284130
	# q3 = -0.8353101960425250
	i=0
	points = []
	for num in nums:
		print("STARTING POINTCLOUD SYNC")
		profi = FrankProfilometer.newProfilometer()
		#DELTA_T = 0.028
		#open profilometer data
		#for i in range(num):
		if data[i].XX==0:
			[XX,YY,ZZ] = profi.ReadPointCloudInputTextFile(filepath+"/PointCloud_"+str(num)+".txt")
			poses = np.loadtxt(filepath + "/egm_poses"+str(num)+".txt")
			timestamp = np.loadtxt(filepath+"/trigger_stamp"+str(num)+".txt")
			data[i].XX = XX
			data[i].YY = YY
			data[i].ZZ = ZZ
			data[i].poses = poses
			data[i].timestamp = timestamp
		else:
			XX = data[i].XX
			YY = data[i].YY
			ZZ = data[i].ZZ
			poses = data[i].poses
			timestamp = data[i].timestamp
			
				
		# t_req = timestamp[1,0]
		# t_resp = timestamp[2,0]
		t_buff = timestamp[0]
		initial = t_buff+DELTA_T # + q0+q1*t_resp+q2*t_buff+q3*(t_resp*t_buff)
		# initial = np.asarray(timestamp[1,0])
		prof_time = []
		for k in range(len(ZZ)):
			prof_time.append(initial+0.01*k)
		###
		## seleect poses close to profilometer timestamp
		###
		prof_time = np.asarray(prof_time)
		res = []
		for j in range(len(prof_time)):
			time = prof_time[j]
			res.append(take_closest(poses[:,7],time))
		index = list(np.asarray(res)[:,1].astype(int))
		#index_after = list(np.asarray(res)[:,3].astype(int))

		# poses_before=poses[index_before]
		# poses_after=poses[index_after]
		poses = poses[index]
		# for i in range(len(prof_time)):
		# 	poses.append(interp_pose(poses_before[i], poses_after[i], prof_time[i]))
		#poses = poses[:,0:7]
		poses = np.asarray(poses)

		Num = np.linspace(0,len(poses),len(poses)+1).astype(int)
		Coord = list(poses[:,0:3])
		Quat_w = list(poses[:,3])
		Quat_x = list(poses[:,4])
		Quat_y = list(poses[:,5])
		Quat_z = list(poses[:,6])

		Quat = list(np.asarray([Quat_w, Quat_x, Quat_y, Quat_z]).T)
		start = 0
		# for l in range(len(Coord)):
		# 	#TO BE REMOVED
		# 	if start==0:
		# 		fCoord =  open(filepath+"/Coords"+str(num)+".txt", "w")
		# 		fCoord.write("Number\tX [mm]\tY [mm]\tZ [mm]\tqw\tqx\tqy\tqz\n") #TO BE REMOVED
		# 		fCoord.close()
		# 		start=1
		
		# 	fCoord = open(filepath+"/Coords"+str(num)+".txt", "a")
		# 	fCoord.write(str(Num[l]) + "\t" + str(Coord[l][0]) + "\t" + str(Coord[l][1]) + "\t"
		# 							+ str(Coord[l][2]) + "\t" + str(Quat[l][0]) + "\t" + str(Quat[l][1]) + "\t"
		# 							+ str(Quat[l][2]) + "\t" + str(Quat[l][3]) + "\n") #TO BE REMOVED
		# fCoord.close()
		start = 0                
		points.append(profi.BuildPointCloud_RobotCoordinate(Num, Coord, Quat, XX, YY, ZZ, SavePath="")) #filepath +"/PointCloud_"+str(num)+".pcd"))
		print("pointcloud sincronized")
		i+=1
	#statepub = rospy.Publisher("/operation_state", OperationState, queue_size =1)
	
	return points
def error_fun(x0, filepath,num2,num1,data):
	global counting
	global index1
	global index2
	all_points  = sync_function_calib(x0,filepath, [num2, num1],data)
	#all_points = #all_points[int(len(all_points)/4)]
	main_view = all_points[0]
	#1_view = 1_view[int(len(1_view)/2):-1] #all_points[0:int(len(all_points)/2)]
	other_view = all_points[1]#all_points[int(len(all_points)/2):-1]
	#other_view = other_view[int(len(other_view)/2):-1] #all_points[0:int(len(all_points)/2)]
	
	#o3d.visualization.draw_geometries([NumpyToPCD(1_view).paint_uniform_color([1,0,0]),NumpyToPCD(other_view).paint_uniform_color([1,1,0]) ])
	if counting==0:
		index1 = np.random.choice(main_view.shape[0],300000, replace=False)
		index2 = np.random.choice(other_view.shape[0], 300000, replace=False)
		counting+=1
	main_view = main_view[index1]
	other_view = other_view[index2]
	      #filter in Z
	bol1 =  np.greater_equal(main_view[:,2],600)
	main_view = main_view[bol1]
	bol2 =  np.greater_equal(other_view[:,2],600)
	other_view = other_view[bol2]
	      #filter in x
	bol1 =  np.greater_equal(main_view[:,0],650)
	main_view = main_view[bol1]
	bol2 =  np.greater_equal(other_view[:,0],650)
	other_view = other_view[bol2]
	#o3d.visualization.draw_geometries([NumpyToPCD(1_view).paint_uniform_color([1,0,0]),NumpyToPCD(other_view).paint_uniform_color([1,1,0]) ])

	tree = KDTree(main_view)
	distances = tree.query(other_view,k=1)[0]
	errors = np.power(np.sum(distances**2)/len(distances),0.5)
	print("error : ",errors)
	print("delta T : ", x0)
	return errors

def CalibrateTimestamp(filepath,statepub, num2, num1):
	global counting
	global index1
	global index2
	print("___Starting Timestamp Optimization___")
	bounds = (-0.1,0.1)
	data = [DataSync.data(),DataSync.data()]
	res = minimize_scalar(error_fun, args=(filepath, num2,num1,data), bounds = bounds, method = "bounded",options={"maxiter" : 10 })
	print("____Finished Timestamp Optimization____")
	print(res.x)
	print(res.fun)
	f = open(filepath+"/delta_t.txt",'w')
	f.write("delta_t : " + str(res.x))
	f.write("\n")
	delta_t = res.x
	counting = 0
	index1 = 0
	index2 = 0
	# msg = OperationState()
	# msg.procedure = 0
	# msg.state = "cluster saved :  " + str(num1)
	# msg.error = "No error"
	# statepub.publish(msg)
	# msg = OperationState()
	# msg.procedure = 0
	# msg.state = "cluster saved :  " + str(num2)
	# msg.error = "No error"
	# statepub.publish(msg)
	return delta_t


def OptimizeXY(data_opt,offset):
	print("ciao")
	tol_lin = 0.1
	x = offset[0]
	y = offset[1]
	x0 = [x,y]
	bound_OX = (x-tol_lin, x+tol_lin)  #boundaries ricerca del minimo
	bound_OY = (y-tol_lin, y+tol_lin)
	args = [data_opt, offset[2]]
	bnds = (bound_OX,bound_OY)
	Min_xy = minimize(ErrorFunctionXY, x0,args = args,method='Powell',bounds = bnds, tol=0.0001)

	# bnds = bound_OX
	# args = [data_opt,Min_y.x,offset[2]]
	# Min_x = minimize_scalar(ErrorFunctionX, args = args,method='bounded',bounds = bnds, tol=0.0001)

	OX = Min_xy.x[0]
	OY=  Min_xy.x[1]
	print('...Finished Parameters Optimization...')
	return OX,OY

# def ErrorFunctionX(x0,data_opt):
# 	print(x0)
# 	OX = x0 #[0:6]
# 	main_view_pts = data_opt[0][0]
# 	main_view_coords =  data_opt[0][1]
# 	main_view_quats =  data_opt[0][2]
	
# 	other_view_pts = data_opt[0][3]
# 	other_view_coords =  data_opt[0][4]
# 	other_view_quats =  data_opt[0][5]

# 	ProfilometerCoordinate = data_opt[0][6]
# 	OY = data_opt[1]
# 	OZ = data_opt[2]

# 	ProfilometerOffset =[OX, OY, OZ]
# 	points0 = Build_point_cloud_fast(main_view_pts,main_view_coords,main_view_quats,ProfilometerCoordinate, ProfilometerOffset)

# 	points = Build_point_cloud_fast(other_view_pts, other_view_coords, other_view_quats, ProfilometerCoordinate, ProfilometerOffset)

# 	errors = []

# 	tree = KDTree(points0)
# 	distances = tree.query(points,k=1)[0]
# 	# trsh = np.percentile(distances,99.9)
# 	# bol = distances<trsh
# 	# distances = distances[bol]      
# 	errors = np.power(np.sum(distances**4)/len(distances), 0.25)          #sum(np.exp2(distances+1))  #RMSE

# 	print(errors)

# 	return errors

def ErrorFunctionXY(x0,data_opt):
	print(x0)
	[OX, OY] = x0 #[0:6]
	main_view_pts = data_opt[0][0]
	main_view_coords =  data_opt[0][1]
	main_view_quats =  data_opt[0][2]
	
	other_view_pts = data_opt[0][3]
	other_view_coords =  data_opt[0][4]
	other_view_quats =  data_opt[0][5]

	ProfilometerCoordinate = data_opt[0][6]
	#OX = data_opt[1]
	OZ = data_opt[1]
	ProfilometerOffset =[OX, OY, OZ]
	points0 = Build_point_cloud_fast(main_view_pts,main_view_coords,main_view_quats,ProfilometerCoordinate, ProfilometerOffset)

	points = Build_point_cloud_fast(other_view_pts, other_view_coords, other_view_quats, ProfilometerCoordinate, ProfilometerOffset)
	#o3d.visualization.draw_geometries([NumpyToPCD(points0).paint_uniform_color([1,0,0]),NumpyToPCD(points).paint_uniform_color([0,0,1])])	
	errors = []

	tree = KDTree(points0)
	distances = tree.query(points,k=1)[0]
	# trsh = np.percentile(distances,99.9)
	# bol = distances<trsh
	# distances = distances[bol]      
	errors = np.power(np.sum(distances**4)/len(distances), 0.25)          #sum(np.exp2(distances+1))  #RMSE

	print(errors)

	return errors



def CalibrateXY(filepath, num2, num3):
	#load and filter all points
	 # Load all the views
	profi = FrankProfilometer.newProfilometer()
	i = 0
	Read=True
	Num = []
	Coord = []
	Quat = []
	XX = []
	YY = []
	ZZ = []

	print('collecting data')
	for n in [num2,num3]:
		Num.append(profi.ReadRobotCoordinateInputFile(filepath+'/Coords'+str(n)+'.txt')[0])
		Coord.append(profi.ReadRobotCoordinateInputFile(filepath+'/Coords'+str(n)+'.txt')[1])
		Quat.append(profi.ReadRobotCoordinateInputFile(filepath+'/Coords'+str(n)+'.txt')[2])
		XX.append(profi.ReadPointCloudInputTextFile(filepath+'/PointCloud_'+str(n)+'.txt')[0])
		YY.append(profi.ReadPointCloudInputTextFile(filepath+'/PointCloud_'+str(n)+'.txt')[1])
		ZZ.append(profi.ReadPointCloudInputTextFile(filepath+'/PointCloud_'+str(n)+'.txt')[2])

	res = []
	for i in range(len(Num)):
			res.append(profi.BuildPointCloud_RobotCoordinate2(Num[i], Coord[i], Quat[i],XX[i], YY[i],ZZ[i],'',profi.ProfilometerCoordSystem,profi.ProfilometerOffset ))
	#  create pointcloud as list of points but also export lsit of Coord ad Quat of robot + Points in Profilometer coordinates, all with the same indexing in order to allow 
	#  filtering with edge detection in pointcloud
	Num = []
	Coord = []
	Quat = []
	XX = []
	YY = []
	ZZ = []
	# filter points with edge_detection and  get indexes
	points_ls = []
	Quat_ls = []
	Coord_ls = []
	Points_prof_ls = []
	for i in range(len(res)):
		points = np.asarray(res[i][0])
		points_prof = np.asarray(res[i][1])
		Coords = np.asarray(res[i][2])
		Quats = np.asarray(res[i][3])
		
	######## Z filtering
	      #filter in Z
		bol =  np.greater_equal(points[:,2],650)
		points = points[bol]
		points_prof = points_prof[bol]
		Coords = Coords[bol]
		Quats = Quats[bol]
		
		#filter in x
		bol =  np.greater_equal(points[:,0],670)
		points = points[bol]
		points_prof = points_prof[bol]
		Coords = Coords[bol]
		Quats = Quats[bol]
		
		

		points_ls.append(points)
		Points_prof_ls.append(points_prof)
		Quat_ls.append(Quats)
		Coord_ls.append(Coords)

	res = []
	points_1 = points_ls[0]
	coords_1 = Coord_ls[0]
	quats_1 = Quat_ls[0]
	profs_1 = Points_prof_ls[0]
	index = np.random.choice(points_1.shape[0], 100000, replace=False)
	points_1 = points_1[index]
	coords_1 = coords_1[index]
	quats_1 = quats_1[index]
	profs_1 = profs_1[index]

	points_2 = points_ls[1]
	coords_2 = Coord_ls[1]
	quats_2 = Quat_ls[1]
	profs_2 = Points_prof_ls[1]
	index = np.random.choice(points_2.shape[0], 20000, replace=False)
	points_2 = points_2[index]
	coords_2 = coords_2[index]
	quats_2 = quats_2[index]
	profs_2 = profs_2[index]
	o3d.visualization.draw_geometries([NumpyToPCD(np.concatenate(points_ls, axis=0)).paint_uniform_color([1,0,0])])	
	data_opt =  [profs_1, coords_1, quats_1, profs_2, coords_2,quats_2, profi.ProfilometerCoordSystem]
	x0 = profi.ProfilometerOffset
	results = OptimizeXY(data_opt, x0)
	return results


def Build_point_cloud_fast(points_ls, coords_ls,quats_ls, ProfilometerCoordinate, ProfilometerOffset, SavePath = '', withNumpy = True): # velocizzare con numpy
	"""
	function
	"""
	points = points_ls
	profiRT = np.array(ProfilometerCoordinate)
	#profiRT = np.transpose(profiRT)
	#print(profiRT)#############################################
	offset = ProfilometerOffset
	zero = [0.0, 0.0, 744.0+95.1]
	points_all = []
	if not withNumpy:
		# slow conversion with for loop
		for i in range(len(points)):
			
			Quat = quats_ls[i]
			cTemp = np.array(coords_ls[i])
			cTemp = np.reshape(cTemp,(1,3))
			RT = np.array(FrankUtilities.Quaternion2RotMatrix(list(Quat))) #Rotation matrix of the Tool
			RT = np.transpose(np.array(RT))
			
			points[i] = points[i].dot(profiRT)+ zero + offset
			points[i] = points[i].dot(RT) + cTemp

	else:
		#speedup super fast the point conversion with numpy
		points = (profiRT@points.T).T + zero+offset
		RT_ls = numpy_quat2RT(quats_ls)        # perchè  fa la trasposta???
		points = np.einsum('ij,ijk->ik', points, RT_ls.T) +  np.array(coords_ls)

	if SavePath!='':
		pcd = o3d.geometry.PointCloud()
		pcd.points = o3d.utility.Vector3dVector(points)
		o3d.io.write_point_cloud(SavePath, pcd)
	return points

def numpy_quat2RT(Quat_ls):
    """
    convert a quaternion list in a Rotation matrix list
    """
    qw=Quat_ls[:,0]
    qx=Quat_ls[:,1]
    qy=Quat_ls[:,2]
    qz=Quat_ls[:,3]
    output = [[np.zeros(len(Quat_ls)),np.zeros(len(Quat_ls)),np.zeros(len(Quat_ls))],
            [np.zeros(len(Quat_ls)),np.zeros(len(Quat_ls)),np.zeros(len(Quat_ls))],
            [np.zeros(len(Quat_ls)),np.zeros(len(Quat_ls)),np.zeros(len(Quat_ls))]]
    output[0][0] = qw**2+qx**2-qy**2-qz**2
    output[0][1] = 2*qx*qy-2*qw*qz
    output[0][2] = 2*qx*qz+2*qw*qy
    output[1][0] = 2*qx*qy+2*qw*qz
    output[1][1] = qw**2-qx**2+qy**2-qz**2
    output[1][2] = 2*qy*qz-2*qw*qx
    output[2][0] = 2*qx*qz-2*qw*qy
    output[2][1] = 2*qy*qz+2*qw*qx
    output[2][2] = qw**2-qx**2-qy**2+qz**2

    return np.asarray(output)