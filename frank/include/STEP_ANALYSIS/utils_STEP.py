from OCC.Core.STEPControl import STEPControl_Reader, STEPControl_Writer, STEPControl_StepModelType, STEPControl_AsIs ,STEPControl_GeometricCurveSet,STEPControl_ShellBasedSurfaceModel
from OCC.Core.IGESControl import IGESControl_Reader 
from OCC.Core.STEPCAFControl import STEPCAFControl_Reader
from OCC.Extend.DataExchange import read_iges_file
from OCC.Core.TopExp import (TopExp_Explorer,
                        topexp_MapShapesAndAncestors,
                        topexp_FirstVertex,
                        topexp_LastVertex)
from OCC.Core.TopAbs import *
from OCC.Core.TopoDS import TopoDS_Shape, topods, topods_Face, topods_Vertex, topods_Edge
from OCC.Core.BRep import BRep_Tool, BRep_Tool_Pnt, BRep_Tool_IsGeometric, BRep_Tool_Parameter, BRep_Tool_Curve
from OCC.Core.BRepMesh import BRepMesh_IncrementalMesh
from OCC.Core.BRepAdaptor import BRepAdaptor_Curve
from OCC.Core.GeomTools import GeomTools_CurveSet
from OCC.Core.BRepBuilderAPI import (
    BRepBuilderAPI_MakeEdge, BRepBuilderAPI_MakeFace
)
from OCC.Core.BRep import BRep_Builder
from OCC.Core.BRepBuilderAPI import (
    BRepBuilderAPI_NurbsConvert, BRepBuilderAPI_MakeWire,BRepBuilderAPI_MakeVertex
)
import sklearn.cluster as cluster
import sklearn.mixture as mixture
from mpl_toolkits.mplot3d import Axes3D

import matplotlib.pyplot as plt
######from OCC.Core.BRepGProp import  SurfaceProperties
from OCC.Core.GeomConvert import geomconvert_SurfaceToBSplineSurface
import numpy as np
import open3d as o3d
from OCC.Display.SimpleGui import init_display
from sklearn.neighbors import BallTree
from OCC.Core.AIS import AIS_Axis, AIS_Point
from OCC.Core.BRepGProp import brepgprop_VolumeProperties, brepgprop_SurfaceProperties,brepgprop_LinearProperties
from OCC.Core.GProp import GProp_GProps
from OCC.Extend.TopologyUtils import TopologyExplorer
from OCC.Core.ShapeAnalysis import *
import math
from mpl_toolkits.mplot3d import Axes3D 
from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeBox, BRepPrimAPI_MakeSphere
from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Fuse
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_MakeEdge
from OCC.Core.BRepMesh import BRepMesh_IncrementalMesh
from OCC.Core.TopExp import TopExp_Explorer
from OCC.Core.TopoDS import TopoDS_Compound, topods_Face, topods_Shell
from OCC.Core.TopAbs import TopAbs_FACE, TopAbs_SHAPE
from OCC.Core.TopLoc import TopLoc_Location
from OCC.Core.gp import gp_Pnt
from OCC.Display.SimpleGui import init_display
import copy
from OCC.Core.StlAPI import StlAPI_Writer
from OCC.Extend.DataExchange import write_stl_file
from sklearn.neighbors import BallTree
from OCC.Core.BRepClass import BRepClass_FaceClassifier
from OCC.Core.BRep import BRep_Tool
from OCC.Core.BRepLib import BRepLib_FindSurface
from OCC.Core.GeomConvert import geomconvert
#from OCC.Core.BRepTools import OuterWire
from OCC.Core.Geom2dConvert  import geom2dconvert
from OCC.Core.BRepExtrema import BRepExtrema_DistShapeShape
#display, start_display, add_menu, add_function_to_menu = init_display()
from sklearn import metrics
from sklearn.datasets import make_blobs
from sklearn.preprocessing import StandardScaler
from OCC.Core.Graphic3d import Graphic3d_ArrayOfPoints
from OCC.Core.Quantity import Quantity_Color
from OCC.Core.Quantity import Quantity_TOC_RGB
from OCC.Core.AIS import AIS_PointCloud
from OCC.Core.StdPrs import StdPrs_ToolRFace 

from sklearn.decomposition import PCA
import os
#from sklearn.neighbors import KDTree
from pykdtree.kdtree import KDTree
from UTILS.FrankCommons import *
from UTILS.FrankUtilities import *
from UTILS.pcd_numpy_utils import *

def STL_to_PCL(filename, n):
    """
    input : filename as .STL file
            n, number of points to be created in the pointcloud
    output: pointcloud as a list of points
    """
    mesh = o3d.io.read_triangle_mesh(filename)
    pointcloud = mesh.sample_points_poisson_disk(n)
    points_ls = PCDToNumpy(pointcloud)
    #DrawPointCloud(Source)
    return points_ls

def Mesh_1_face(face, dis):
    """ Creates a list of  points from a bspline surface
    input: face = topods_face
            dis = distance between each point that will be created on the face
    """

    surface = BRep_Tool.Surface(face)
    pnts = []
    sas = ShapeAnalysis_Surface(surface)
    umin, umax, vmin, vmax = shapeanalysis_GetFaceUVBounds(face)
    u = umin
    while u < umax:
        v = vmin
        while v < vmax:
            p = sas.Value(u, v)
            if appartiene_a_faccia(p,face,0.1):
                pnts.append([p.X(),p.Y(),p.Z()])
            v += dis
        u += dis
    return pnts

def rotation_from_3points(center,normal,vector):
    '''
    input: center points of the TCP== hole location (mid in axis parsing), points normal to the surface (last in axis parsing), 
    vector of 1st pca of the cluster to determine the others tcp components
    output: list of TCP orientation expressed in rotation matrix RT, quaternions Quat, and Euler angles Eul
    '''

    a = np.squeeze(np.asarray(normal))
    b = np.squeeze(np.asarray(center))  #center of the new coordinate system
    
    Z = -(a-b) #vector going from mid to outer normal    check if is the correct DIRECTION OR THE REVERSE!!!
    # if len(Z)==3:
    #     norm = np.linalg.norm(Z)
    # else:
    norm = np.reshape(np.linalg.norm(Z,axis=1),(len(Z),1))
    Z= Z/norm
    temp_v = np.ones((len(Z),1))*vector
    Versor2 = np.cross(Z,temp_v)
    norm = np.reshape(np.linalg.norm(Versor2,axis=1),(len(Versor2),1))
    Versor2 = Versor2/norm
    #Versor1 = np.asarray(vector)
    #norm = np.reshape(np.linalg.norm(Versor1,axis=1),(len(Versor1),1))
    #Versor1 = Versor1/norm
    RT = []
    Eul = []
    Quat = []
    RT2 = []
    Eul2 = []
    Quat2 = []
    for i in range(len(Z)):
        TargetNormal = list(Z[i])
        VerticalVersor = list(Versor2[i])
        ToolLength = 0
        TargetPoint = list(b[i])
        result = Target3rdVersor2PosRT(TargetPoint, TargetNormal, VerticalVersor, ToolLength)
        #result2 = Target3rdVersor2PosRT(TargetPoint, TargetNormal, -VerticalVersor, ToolLength)
        #result = Target2PosRT(TargetPoint, TargetNormal, ToolLength)
        ToolRT = result[1]
        RT.append(ToolRT)
        ToolEul = RotMatrix2Euler(ToolRT)
        Eul.append(ToolEul)
        ToolQuat = RotMatrix2Quaternion(ToolRT)
        Quat.append(ToolQuat)

        ToolRT2 = result[2]
        RT2.append(ToolRT2)
        ToolEul2 = RotMatrix2Euler(ToolRT2)
        Eul2.append(ToolEul2)
        ToolQuat2 = RotMatrix2Quaternion(ToolRT2)
        Quat2.append(ToolQuat2)

    return Quat,RT,Eul,Quat2,RT2,Eul2

def offset_along_vector(pts,v,distance):
    '''
    input: list of points (or just 1), vector(s) in wich direction
            you want to generate the offsetted point np.array(n,3) or (3,1) if just 1, distance in mm
    output: list of offsetted point np.array()
    '''
    if len(v)==3:
        norm = np.linalg.norm(v)
    else:
        norm = np.reshape(np.linalg.norm(v,axis=1),(len(v),1))
    v= v/norm
    off_pts = pts+distance*v
    return off_pts

def draw_result(source, target):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    o3d.visualization.draw_geometries([source_temp, target_temp])

def STL_save(shape,filename):
	write_stl_file(shape,filename, mode="binary", linear_deflection=0.5, angular_deflection=0.3)

def cluster_points_on_face(pts_ls,face_ls):
    ''' pts: list of points to be clusterized in 
        each face
        faces: list of pc 
    '''
    tol = 0.5
    clusters  = [[]for n in range(len(face_ls))]  # inizialize point clusters array
    count = 0
    for j in range(len(pts_ls)):
        p= pts_ls[j]
        for i in range(len(face_ls)):
            f = face_ls[i]
            if appartiene_a_faccia(p,f,tol):
                clusters[i].append(p)
                count+=1
            print(count)
        #f = face_ls[len(face_ls)-i-1]
        ##res = appartiene_a_faccia(p, f,tol)
        #shape = convert_topods_in_shape(f)
        #face_pts = convert_shape_to_points(shape,5000)
        #face_pts= np.asarray(face_pts)
        #tree = BallTree(face_pts)
        ##point =  np.asarray([point])
        #distance, index = tree.query(pts_ls,k=1)
        #bol = distance<tol
        #bol = np.reshape(bol,(len(bol),))
        #if max(bol)==1:
        #    if len(pts_ls[bol])>=4:
        #        clusters[i].append(pts_ls[bol])
        #        pts_ls = pts_ls[~bol]
        #print(len(pts_ls))
        #print(i)
        #if len(pts_ls)<2:
        #    break
    return clusters

def appartiene_a_faccia(point,face,tol):
    """
    check if a point belong to a TopoDS_face
    return True if it does
    """
    if type(point) is list:
        gp_pnt = gp_Pnt()
        gp_pnt.SetX(point[0])
        gp_pnt.SetY(point[1])
        gp_pnt.SetZ(point[2])
    else:
        gp_pnt = point
    vertex = BRepBuilderAPI_MakeVertex(gp_pnt).Vertex()
    d= Distance_shape2shape(vertex,face)
    #shape = convert_topods_in_shape(face)
    #face_pts = convert_shape_to_points(shape,2000)
    #face_pts= np.asarray(face_pts)
    #tree = BallTree(face_pts)
    #point =  np.asarray([point])
    #distance, index = tree.query(point,k=1)
    #print(distance)
    if d<tol:
        return True
    else:
        return False

def Distance_shape2shape(shape1,shape2):
    """
    returns euclidian distance between 2 topods_shape
    """
    d = BRepExtrema_DistShapeShape(shape1,shape2).Value()  
   # print(d)
    return d

def convert_topods_in_shape(topods):
    shape = TopoDS_Shape(topods)
    return shape

def convert_shape_to_points(shape,n_points):
    '''
    save the shape as stl
    import the
    convert stl to pc with open3d
    '''
    temp_file = "E:/temp.stl"
   # BRepMesh_IncrementalMesh(shape, 0.1)
    write_stl_file(shape, temp_file, mode="binary", linear_deflection=0.05, angular_deflection=0.3)
    points = STL_to_PCL(temp_file, n_points) 
    points = list(points)
    return points

def display_shape(shape):

    display, start_display, add_menu, add_function_to_menu = init_display()
    display.EraseAll()
    display.DisplayShape(shape)
    start_display()

def from_point_to_gp_point(points):
    """
    convert np.array(N,3) list of points in opencascade gp_point
    """
    points_3d = Graphic3d_ArrayOfPoints(len(points), True)
    gp_points= []
    for p in points:
        gp_point = gp_Pnt()
        gp_point.SetX(p[0])
        gp_point.SetY(p[1])
        gp_point.SetZ(p[2])
        gp_points.append(gp_point)
        color = Quantity_Color(1,0,0,Quantity_TOC_RGB)
        points_3d.AddVertex(gp_point,color)
    point_cloud = AIS_PointCloud()
    point_cloud.SetPoints(points_3d)

    return points_3d, point_cloud, gp_points

def BuildCompoundFromShapes(shape_ls):
    """
    input: list of topods_shape
    output : topods_compound
    """
    builder = BRep_Builder()
    Comp = TopoDS_Compound()
    builder.MakeCompound(Comp)
    for s in shape_ls:
        s = convert_topods_in_shape(s)
        builder.Add(Comp,s)
    return Comp

def STEP_open(filename):
        '''
        STEP file reader
        return: topds_shape
        '''
        step_reader = STEPControl_Reader()

        step_reader.ReadFile(filename)
        step_reader.TransferRoot()
   
        shape = step_reader.Shape()

        return shape



def get_shells(_shape):

    """ return the faces from `_shape`

    :param _shape: TopoDS_Shape
    :return: a list of shells found in `_shape`
                a list of surface extensions
    """
    topExp = TopExp_Explorer()
    topExp.Init(_shape, TopAbs_SHELL)
    _shells = []
    surfaces = []
    while topExp.More():
        fc = topods_Shell(topExp.Current())
        _shells.append(fc)
        area = face_dimension(fc)
        surfaces.append(area)
        topExp.Next()

    return _shells, surfaces

def get_faces(_shape):
    """ return the faces from `_shape`

    :param _shape: TopoDS_Shape, or a subclass like TopoDS_Solid
    :return: a list of faces found in `_shape`
                a list of surface extentions
    """
    topExp = TopExp_Explorer()
    topExp.Init(_shape, TopAbs_FACE)
    _faces = []
    surfaces = []
    while topExp.More():
        fc = topods_Face(topExp.Current())
        _faces.append(fc)
        area = face_dimension(fc)
        surfaces.append(area)
        topExp.Next()

    return _faces, surfaces

def get_vertex(_shape):
    """ return the faces from `_shape`

    :param _shape: TopoDS_Shape, or a subclass
    :return: a list of topds_vertex found in `_shape`, and list of points (x,y,z)
    """
    topExp = TopExp_Explorer()
    topExp.Init(_shape, TopAbs_VERTEX)
    _vertexes = []
    _vertexes_ls = []
    while topExp.More():
        vc = topods_Vertex(topExp.Current())
        _vertexes.append(vc)
        vc_pts = [BRep_Tool().Pnt(vc).Coord(1),BRep_Tool().Pnt(vc).Coord(2),BRep_Tool().Pnt(vc).Coord(3)]
        _vertexes_ls.append(vc_pts)
        topExp.Next()

    return _vertexes, _vertexes_ls

def get_edges(_shape):
    """ return the faces from `_shape`

    :param _shape: TopoDS_Shape, or a subclass like TopoDS_Solid
    :return: a list of faces found in `_shape`
    """
    topExp = TopExp_Explorer()
    topExp.Init(_shape, TopAbs_EDGE)
    #t = TopologyExplorer(_shape)

    _edges=[]
    ##_vertexes_ls = []
    while topExp.More():
        fc = topods_Edge(topExp.Current())
        _edges.append(fc)
        topExp.Next()
    #_edges = t.edges()

    return _edges

def get_shapes(compound):
    """ return the faces from `_shape`

    :param _shape: TopoDS_Shape, or a subclass like TopoDS_Solid
    :return: a list of faces found in `_shape`
    """
    topExp = TopExp_Explorer()
    topExp.Init(compound, TopAbs_SHAPE)
    shapes = []
    while topExp.More():
        shp = TopoDS_Shape(topExp.Current())
        shapes.append(shp)
        topExp.Next()
    return shapes

def edge_length(_edge):
    '''
    return length of topods_edge
    '''
    props = GProp_GProps()
    brepgprop_LinearProperties(_edge,props)
    length = props.Mass()
    return length

def face_dimension(_face):
    '''
    return surface of topods_face
    '''
    props = GProp_GProps()
    brepgprop_SurfaceProperties(_face,props)
    area = props.Mass()
    return area

def Distance_point2shape(topds_point,topds_shape):
    '''
    return distance of point to general topods_shape
    '''
    d = BRepExtrema_DistShapeShape(topds_point,topds_shape).Value()  
    return d





