import numpy as np
import open3d as o3d
#import pyransac3d
from scipy.optimize import minimize
import pandas as PD
from pyntcloud import PyntCloud 

def DetectMultiPlanes(points, min_ratio=0.05, threshold=0.5, iterations=1000):
    """ Detect multiple planes from given point clouds

    Args:
        points (np.ndarray): 
        min_ratio (float, optional): The minimum left points ratio to end the Detection. Defaults to 0.05.
        threshold (float, optional): RANSAC threshold in (m). Defaults to 0.01.

    Returns:
        [List[tuple(np.ndarray, List)]]: Plane equation and plane point index
    """

    plane_list = []
    N = len(points)
    target = points.copy()
    count = 0

    while count < (1 - min_ratio) * N:
        w, index = PlaneRegression(
            target, threshold=threshold, init_n=3, iter=iterations)
    
        count += len(index)
        plane_list.append((w, target[index]))
        target = np.delete(target, index, axis=0)

    return plane_list

def ReadPlyPoint(fname):
    """ read point from ply

    Args:
        fname (str): path to ply file

    Returns:
        [ndarray]: N x 3 point clouds
    """

    pcd = o3d.io.read_point_cloud(fname)

    return PCDToNumpy(pcd)


def NumpyToPCD(xyz):
    """ convert numpy ndarray to open3D point cloud 

    Args:
        xyz (ndarray): 

    Returns:
        [open3d.geometry.PointCloud]: 
    """

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz)

    return pcd


def PCDToNumpy(pcd):
    """  convert open3D point cloud to numpy ndarray

    Args:
        pcd (open3d.geometry.PointCloud): 

    Returns:
        [ndarray]: 
    """

    return np.asarray(pcd.points)


def RemoveNan(points):
    """ remove nan value of point clouds

    Args:
        points (ndarray): N x 3 point clouds

    Returns:
        [ndarray]: N x 3 point clouds
    """

    return points[~np.isnan(points[:, 0])]


def RemoveNoiseStatistical(pc, nb_neighbors=20, std_ratio=2.0):
    """ remove point clouds noise using statitical noise removal method

    Args:
        pc (ndarray): N x 3 point clouds
        nb_neighbors (int, optional): Defaults to 20.
        std_ratio (float, optional): Defaults to 2.0.

    Returns:
        [ndarray]: N x 3 point clouds
    """

    pcd = NumpyToPCD(pc)
    cl, ind = pcd.remove_statistical_outlier(
        nb_neighbors=nb_neighbors, std_ratio=std_ratio)

    return PCDToNumpy(cl)


def DownSample(pts, voxel_size=0.003):
    """ down sample the point clouds

    Args:
        pts (ndarray): N x 3 input point clouds
        voxel_size (float, optional): voxel size. Defaults to 0.003.

    Returns:
        [ndarray]: 
    """

    p = NumpyToPCD(pts).voxel_down_sample(voxel_size=voxel_size)

    return PCDToNumpy(p)


def PlaneRegression(points, threshold=0.01, init_n=3, iter=1000):
    """ plane regression using ransac

    Args:
        points (ndarray): N x3 point clouds
        threshold (float, optional): distance threshold. Defaults to 0.003.
        init_n (int, optional): Number of initial points to be considered inliers in each iteration
        iter (int, optional): number of iteration. Defaults to 1000.

    Returns:
        [ndarray, List]: 4 x 1 plane equation weights, List of plane point index
    """

    pcd = NumpyToPCD(points)

    w, index = pcd.segment_plane(
        threshold, init_n, iter)

    return w, index




def DrawResult(points, colors):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(colors)
    o3d.visualization.draw_geometries([pcd])

def DrawPointCloud(points):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    o3d.visualization.draw_geometries([pcd])

def BestFitPlane(points):
    x = Plane3Points(points[0],points[int(len(points)/2)],points[len(points)-1])

    bnds = ((-100,100),(-100,100),(-100,100),(-1000,1000))
    args = (points)
    min = minimize(error_fun,x,args = args, method = 'Powell',tol=0.00000000001)
    [a,b,c,d] = min.x
    return [a,b,c,d]
def error_fun(x,points):
    A = x[0]
    B = x[1]
    C = x[2]
    D = x[3]
    d = abs((A*points[:,0]+B*points[:,1]+C*points[:,2]+D))/(np.sqrt(A*A + B*B+C*C))
    print(np.nansum(d))
    return np.nansum(d)
def Plane3Points(p1,p2,p3):
    u = p1- p3
    v = p2-p3
    #vettore normale
    cp = np.cross(u,v)
    A,B,C = np.cross(u,v)
    #n /= np.linalg.norm(n)
    #A = n[:, 0]
    #B = n[:, 1]
    #C = n[:, 2]
    D = -np.dot(cp,p3)
    return [A,B,C,D]
    
def FilterWorkingVolume(points):
    '''
    input: pointcloud
    output: filtered pointcloud
    '''
    #just Z filter
    bol = points[:,2] > 50
    points = points[bol]
    pcd = NumpyToPCD(points)
    # pcd = pcd.uniform_down_sample(every_k_points=5)
   # print("radious outlier removal : ")

    # cl, ind = pcd.remove_radius_outlier(nb_points=100, radius=30)
    # #print("statistical outlier removal")
    # pcd = pcd.select_by_index(ind)
    cl, ind = pcd.remove_statistical_outlier(nb_neighbors=50,
                                                    std_ratio=0.1)
    pcd = pcd.select_by_index(ind)
    
   # pcd = pcd.uniform_down_sample(every_k_points=3)

    points = PCDToNumpy(pcd)

    return points

def Edge_Detection(points, k_n, thresh):
    #pcd1 = PyntCloud.from_file("/ArtificialPointClouds/bunny.pcd")
    #pcd1 = PyntCloud.from_file("/TetrahedronMultiple.pcd")
    #pcd1 = PyntCloud.from_file("/ArtificialPointClouds/CubeFractal2.pcd")
    #output_dir = "./detected_edge/"

    #if not os.path.exists(output_dir):
    #    os.makedirs(output_dir)
    clmns = ['x','y','z']
    points_pd = PD.DataFrame(data=points,columns=clmns)
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
    pcd_pd =PD.DataFrame(data=pcd_np,columns=clmns)
    pcd_pd['red'] = sigma_value.astype(np.uint8)

    #pcd_points = PyntCloud(pd.DataFrame(data=pcd_np,columns=clmns))
    pcd_points = PyntCloud(pcd_pd)
    edge_points = PyntCloud(PD.DataFrame(data=edge_np,columns=clmns))

    # pcd_points.plot()
    # edge_points.plot()

    #PyntCloud.to_file(pcd_points,output_dir+'pointcloud_edges.ply')   # Save the whole point cloud by painting the edge points
    #PyntCloud.to_file(edge_points,output_dir+'edges.ply')             # Save just the edge points
    return edge_np[:,0:3] ,np.where(pcd_np[:,3] == 0)