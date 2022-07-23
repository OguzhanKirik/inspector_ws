import sys
from pathlib import Path
main_dir = str(Path(__file__).resolve().parent.parent)
include_dir1 = str(Path(__file__).resolve().parent)

sys.path.append(main_dir)
sys.path.append(include_dir1)

print(main_dir)

from utils_STEP import *
from OCC.Display.SimpleGui import init_display
import numpy as np
from UTILS.pcd_numpy_utils import *
from joblib import Parallel, delayed
import time
import multiprocessing as mp
import random
import open3d as o3d


def meshwithOCC(source_file,save_file = ''):
	'''
	input: STEP file path
	output: pointcloud as numpy array
	'''
	#### open file
	shape = STEP_open(source_file)
	#extract all faces
	faces, surfaces = get_faces(shape)
	index =  list(np.argsort(surfaces))
	faces = np.asarray(faces)
	faces = faces[index]
	
	all_pts = []
	t = time.time()
	count = 0
	N = mp.cpu_count()
	random.shuffle(faces)
	# divide faces in N of processor
	six_faces = np.array_split(faces, N)
	pool =mp.Pool(N)
	points = pool.map(mesh_parallel,six_faces)
	print(time.time() - t)
	points = np.asarray(points)
	points = np.concatenate(points,axis=0)
	points = list(filter(None, points))
	points = np.concatenate(points,axis=0)
	print(len(points))
	DrawPointCloud(points)
	pcd = NumpyToPCD(points)
	if save_file!='':
		o3d.io.write_point_cloud(save_file,pcd)
	return points

def mesh_parallel(faces):
	"""
	help function that parallelize process of face pointcloud generation
	"""
	all_pts = []
	cnt = 0
	for f in faces:
		pts = Mesh_1_face(f,10)
		all_pts.append(pts)
		cnt+=1
		print(cnt)
	return all_pts
#
def meshwithSTL(source_file, npoints, save_file=''):
	"""
	input: source_file is the .STEP file from which we need to generate the pointcloud
		   npoints total number of points that will be in the pointcloud
		   save file, .ply file that will be outputted
	output: list of points as np.array(N,3)
	"""
	shape = STEP_open(source_file)

	t = time.time()
	STL_save(shape,main_dir+'/temp.stl')
	all_pts = STL_to_PCL(main_dir+'/temp.stl',npoints)

	print(time.time() - t)
	DrawPointCloud(all_pts)
	pcd1 = NumpyToPCD(all_pts)
	if save_file!='':
		o3d.io.write_point_cloud(save_file,pcd1) 
	return all_pts



#meshwithOCC(main_dir+'/data/mockup_fori_colori4.stp')
meshwithSTL(main_dir+'/data/Skullcap Hole Identification_allaxis_colors.stp',1000000, main_dir+'/data/Skullcap_pointcloud.pcd' )
	#PLY_to_PCD()