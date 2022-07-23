#! /usr/bin/env python3

from OCC.Core.STEPControl import STEPControl_Reader, STEPControl_Writer, STEPControl_StepModelType, STEPControl_AsIs ,STEPControl_GeometricCurveSet,STEPControl_ShellBasedSurfaceModel
from OCC.Core.IGESControl import IGESControl_Reader , IGESControl_Writer
from OCC.Core.STEPCAFControl import STEPCAFControl_Reader
from OCC.Core.XCAFDoc import XCAFDoc_DocumentTool_ShapeTool, XCAFDoc_DocumentTool_ColorTool
from OCC.Core.Interface import Interface_Static_SetCVal
from OCC.Core.IFSelect import IFSelect_RetDone, IFSelect_ItemsByEntity
from OCC.Extend.DataExchange import read_iges_file
from OCC.Core.TDocStd import TDocStd_Document
from OCC.Core.TDF import TDF_LabelSequence, TDF_Label
from OCC.Core.TCollection import TCollection_ExtendedString
from OCC.Core.Quantity import Quantity_Color, Quantity_TOC_RGB
from OCC.Core.TopLoc import TopLoc_Location
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCC.Extend.TopologyUtils import discretize_edge, get_sorted_hlr_edges, list_of_shapes_to_compound
from OCC.Core.TopExp import (TopExp_Explorer,
                        topexp_MapShapesAndAncestors,
                        topexp_FirstVertex,
                        topexp_LastVertex)
from OCC.Core.TopAbs import *
from OCC.Core.TopoDS import TopoDS_Shape, topods, topods_Face, topods_Vertex, topods_Edge, TopoDS_Compound, topods_Shell, TopoDS_Edge
from OCC.Core.BRep import BRep_Tool, BRep_Tool_Pnt, BRep_Tool_IsGeometric, BRep_Tool_Parameter, BRep_Tool_Curve, BRep_Builder
from OCC.Core.BRepMesh import BRepMesh_IncrementalMesh
from OCC.Core.BRepAdaptor import BRepAdaptor_Curve
from OCC.Core.GeomTools import GeomTools_CurveSet
from OCC.Core.BRepBuilderAPI import (
    BRepBuilderAPI_MakeEdge, BRepBuilderAPI_MakeFace,BRepBuilderAPI_NurbsConvert, BRepBuilderAPI_MakeWire,BRepBuilderAPI_MakeVertex
)
from OCC.Core.Bnd import Bnd_Box2d
from OCC.Core.StlAPI import stlapi_Read, StlAPI_Writer
import os
import sklearn.cluster as cluster
import sklearn.mixture as mixture
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from OCC.Core.GeomConvert import geomconvert_SurfaceToBSplineSurface
import numpy as np
import open3d as o3d
from sklearn.neighbors import BallTree
from OCC.Core.AIS import AIS_Axis, AIS_Point,AIS_PointCloud
from OCC.Core.BRepGProp import brepgprop_VolumeProperties, brepgprop_SurfaceProperties,brepgprop_LinearProperties
from OCC.Core.GProp import GProp_GProps
from OCC.Extend.TopologyUtils import TopologyExplorer
from OCC.Core.ShapeAnalysis import *
import math
from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeBox, BRepPrimAPI_MakeSphere
from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Fuse
from OCC.Core.TopLoc import TopLoc_Location
from OCC.Core.gp import gp_Pnt, gp_Dir, gp_Pnt2d
from OCC.Display.SimpleGui import init_display
import copy
from OCC.Core.StlAPI import StlAPI_Writer
from OCC.Extend.DataExchange import write_stl_file
from sklearn.neighbors import BallTree
from OCC.Core.BRepClass import BRepClass_FaceClassifier
from OCC.Core.BRep import BRep_Tool
from OCC.Core.BRepLib import BRepLib_FindSurface
from OCC.Core.GeomConvert import geomconvert
from OCC.Core.Geom2dConvert  import geom2dconvert
from OCC.Core.BRepExtrema import BRepExtrema_DistShapeShape
from sklearn import metrics
from sklearn.datasets import make_blobs
from sklearn.preprocessing import StandardScaler
from OCC.Core.Graphic3d import Graphic3d_ArrayOfPoints
from OCC.Core.Quantity import Quantity_Color
from OCC.Core.Quantity import Quantity_TOC_RGB
from OCC.Core.StdPrs import StdPrs_ToolRFace 
import sys
from pathlib import Path
include_dir = str(Path(__file__).resolve().parent)
sys.path.append(include_dir)
print(include_dir)
from utils_STEP import *
from UTILS.pcd_numpy_utils import *
import random
from skimage.measure import LineModelND, ransac, approximate_polygon,EllipseModel
import scipy
from pykdtree.kdtree import KDTree
#   
# ####################################################################
#####   StepAnalyzer: 
#####   Module for handling the STEP file opening and data extraction ######
########################################################################    
#
#



class StepAnalyzer():
    def STEP_save(self,shape,filename):
        '''
        input: TopoDS_Shape, general opencascade format to handle geometries, filename to be saved on in .stp
        output: /
        '''
        writer = STEPControl_Writer()
        writer.Transfer(shape,STEPControl_AsIs)
        writer.Write(filename)
    
    def STEP_open_with_shapes_colors(self,filename):
        """ Returns list of topods_shape and corresponding colors
        allow to identify shape and subshapes
        input: filename .stp file that needs to be analyzed
        output: list of Topods_shape, list of rgb colors [r,g,b] float   
        """
        if not os.path.isfile(filename):
            raise FileNotFoundError("%s not found." % filename)
        # the list:
        output_shapes = []
        colors = []
        # create an handle to a document
        doc = TDocStd_Document(TCollection_ExtendedString("pythonocc-doc"))

        # Get root assembly
        shape_tool = XCAFDoc_DocumentTool_ShapeTool(doc.Main())
        color_tool = XCAFDoc_DocumentTool_ColorTool(doc.Main())
    
        step_reader = STEPCAFControl_Reader()
        step_reader.SetColorMode(True)
        step_reader.SetLayerMode(True)
        step_reader.SetNameMode(True)
        step_reader.SetMatMode(True)
        step_reader.SetGDTMode(True)

        status = step_reader.ReadFile(filename)
        if status == IFSelect_RetDone:
            step_reader.Transfer(doc)

        locs = []

        def _get_sub_shapes(lab, loc):
            print("Is Assembly    :", shape_tool.IsAssembly(lab))
            print("Is Free        :", shape_tool.IsFree(lab))
            print("Is Shape       :", shape_tool.IsShape(lab))
            print("Is Compound    :", shape_tool.IsCompound(lab))
            print("Is Component   :", shape_tool.IsComponent(lab))
            print("Is SimpleShape :", shape_tool.IsSimpleShape(lab))
            print("Is Reference   :", shape_tool.IsReference(lab))

            #users = TDF_LabelSequence()
            #users_cnt = shape_tool.GetUsers(lab, users)
            #print("Nr Users       :", users_cnt)

            l_subss = TDF_LabelSequence()
            shape_tool.GetSubShapes(lab, l_subss)
            print("Nb subshapes   :", l_subss.Length())
            l_comps = TDF_LabelSequence()
            shape_tool.GetComponents(lab, l_comps)
            print("Nb components  :", l_comps.Length())
       
            name = lab.GetLabelName()
            print("Name :", name)

            if shape_tool.IsAssembly(lab):
                l_c = TDF_LabelSequence()
                shape_tool.GetComponents(lab, l_c)
                for i in range(l_c.Length()):
                    label = l_c.Value(i+1)
                    if shape_tool.IsReference(label):
                        label_reference = TDF_Label()
                        shape_tool.GetReferredShape(label, label_reference)
                        loc = shape_tool.GetLocation(label)
                        locs.append(loc)
                        _get_sub_shapes(label_reference, loc)
                        locs.pop()
            #if shape_tool.IsCompound(lab):

            elif shape_tool.IsSimpleShape(lab):
                shape = shape_tool.GetShape(lab)

                loc = TopLoc_Location()
                for l in locs:
                    loc = loc.Multiplied(l)
                c = Quantity_Color(0.5, 0.5, 0.5, Quantity_TOC_RGB)  # default color
                colorSet = False
                if (color_tool.GetInstanceColor(shape, 0, c) or
                        color_tool.GetInstanceColor(shape, 1, c) or
                        color_tool.GetInstanceColor(shape, 2, c)):
                    color_tool.SetInstanceColor(shape, 0, c)
                    color_tool.SetInstanceColor(shape, 1, c)
                    color_tool.SetInstanceColor(shape, 2, c)
                    colorSet = True
                    n = c.Name(c.Red(), c.Green(), c.Blue())
                    print('    instance color Name & RGB: ', c, n, c.Red(), c.Green(), c.Blue())

                if not colorSet:
                    if (color_tool.GetColor(lab, 0, c) or
                            color_tool.GetColor(lab, 1, c) or
                            color_tool.GetColor(lab, 2, c)):

                        color_tool.SetInstanceColor(shape, 0, c)
                        color_tool.SetInstanceColor(shape, 1, c)
                        color_tool.SetInstanceColor(shape, 2, c)

                        n = c.Name(c.Red(), c.Green(), c.Blue())
                        print('    shape color Name & RGB: ', c, n, c.Red(), c.Green(), c.Blue())

                shape_disp = BRepBuilderAPI_Transform(shape, loc.Transformation()).Shape()
                #if not shape_disp in output_shapes:
                #    output_shapes[shape_disp] = [lab.GetLabelName(), c]
                output_shapes.append(shape_disp)
                colors.append([c.Red(), c.Green(), c.Blue()])

                for i in range(l_subss.Length()):
                    lab_subs = l_subss.Value(i+1)
                    #print("\n########  simpleshape subshape label :", lab)
                    shape_sub = shape_tool.GetShape(lab_subs)

                    c = Quantity_Color(0.5, 0.5, 0.5, Quantity_TOC_RGB)  # default color
                    colorSet = False
                    if (color_tool.GetInstanceColor(shape_sub, 0, c) or
                            color_tool.GetInstanceColor(shape_sub, 1, c) or
                            color_tool.GetInstanceColor(shape_sub, 2, c)):
                        color_tool.SetInstanceColor(shape_sub, 0, c)
                        color_tool.SetInstanceColor(shape_sub, 1, c)
                        color_tool.SetInstanceColor(shape_sub, 2, c)
                        colorSet = True
                        n = c.Name(c.Red(), c.Green(), c.Blue())
                        print('    instance color Name & RGB: ', c, n, c.Red(), c.Green(), c.Blue())

                    if not colorSet:
                        if (color_tool.GetColor(lab_subs, 0, c) or
                                color_tool.GetColor(lab_subs, 1, c) or
                                color_tool.GetColor(lab_subs, 2, c)):
                            color_tool.SetInstanceColor(shape, 0, c)
                            color_tool.SetInstanceColor(shape, 1, c)
                            color_tool.SetInstanceColor(shape, 2, c)

                            n = c.Name(c.Red(), c.Green(), c.Blue())
                            print('    shape color Name & RGB: ', c, n, c.Red(), c.Green(), c.Blue())
                    shape_to_disp = BRepBuilderAPI_Transform(shape_sub, loc.Transformation()).Shape()
                    # position the subshape to display
                    #if not shape_to_disp in output_shapes:
                    #    output_shapes[shape_to_disp] = [lab_subs.GetLabelName(), c]
                    output_shapes.append(shape_to_disp)
                    colors.append([c.Red(), c.Green(), c.Blue()])
        def _get_shapes():
            labels = TDF_LabelSequence()
            shape_tool.GetFreeShapes(labels)
            print()
            print("Number of shapes at root :", labels.Length())
            print()
            for i in range(labels.Length()):
                root_item = labels.Value(i+1)
                _get_sub_shapes(root_item, None)
        _get_shapes()

        return output_shapes, colors

    def find_axis_color(self,filename, color_list):
        '''
        input: name of the step file, list of color of the axis to find in step [r,g,b]
        output: list of middle, first and last point of each axis as np.array(N,3)

        '''
        shape_ls, colors = self.STEP_open_with_shapes_colors(filename)
        bol = []

        for i in range(len(colors)):
            c = colors[i]
            if c in color_list:
                e = get_edges(shape_ls[i])
                if e:
                    bol.append(1)
            else: bol.append(0) 
        shape_ls = [shape_ls[i] for i in range(len(shape_ls)) if bol[i]] # extract all of the shape with the specified color
        colors = [colors[i] for i in range(len(colors)) if bol[i]]
        #EXTRACT FROM ALL THE SHAPES JUST THE AXIS
        # shape = BuildCompoundFromShapes(shape_ls)
        # edges_ls = get_edges(shape)
        first_pts_ls = []
        last_pts_ls = []
        count = 0
        for e in shape_ls:   
           #
            first = topexp_FirstVertex(e)
            if not first.IsNull(): # remove closed edge
                first_pts = [BRep_Tool().Pnt(first).Coord(1),BRep_Tool().Pnt(first).Coord(2),BRep_Tool().Pnt(first).Coord(3)]
                first_pts_ls.append(first_pts)
                last = topexp_LastVertex(e)
                last_pts = [BRep_Tool().Pnt(last).Coord(1),BRep_Tool().Pnt(last).Coord(2),BRep_Tool().Pnt(last).Coord(3)]
                last_pts_ls.append(last_pts)
                print(count)
                count+=1
        first_pts_ls = np.array(first_pts_ls)
        last_pts_ls = np.array(last_pts_ls)
        first_pc = NumpyToPCD(first_pts_ls)
        last_pc = NumpyToPCD(last_pts_ls)
        colors = np.array(colors)
        last_pc.paint_uniform_color([1, 0, 0])
        m_ls= (first_pts_ls + last_pts_ls)/2
        m_pc = NumpyToPCD(m_ls)
        m_pc.paint_uniform_color([1, 1, 0])
        m_ls= (first_pts_ls + last_pts_ls)/2
        m_ls = np.array(m_ls)
        # controllo i punti uguali
        ml_uniq,index = np.unique(m_ls,return_index = True,axis=0)
        m_ls = m_ls[index]
        first_pts_ls = first_pts_ls[index]
        last_pts_ls =last_pts_ls[index]
        colors_out = colors[index]
        #visualization
        m_pc = NumpyToPCD(m_ls)
        m_pc.paint_uniform_color([1, 1, 0])
        first_pc = NumpyToPCD(first_pts_ls)
        first_pc.paint_uniform_color([1, 0,1])
        last_pc = NumpyToPCD(last_pts_ls)
        last_pc.paint_uniform_color([1, 0, 0])
       # o3d.visualization.draw_geometries([first_pc, last_pc,m_pc])
        ###################################################
        #### check if Z direction is always the same ######
        ###################################################
        tree = KDTree(np.squeeze(m_ls))                         
        for i in range(len(m_ls)):
            p = m_ls[i]                                                     #
            ax_p = last_pts_ls[i]-p                                         #   check if all neighbour axis are oriented in the same direction
            indexes  = tree.query(np.reshape(p,(1,3)), k=3)[1]              #   inverts the one single ones in the opposite direction
            nn_mid = m_ls[indexes]                                          #
            nn_last = last_pts_ls[indexes]
            ax_vects = nn_last - nn_mid
            avg_dir = np.sum(np.squeeze(ax_vects),axis=0)/len(ax_vects)
            thrs = 0
            if ax_p@avg_dir<thrs:
                temp = copy.deepcopy(first_pts_ls[i])
                first_pts_ls[i] = last_pts_ls[i]
                last_pts_ls[i] = temp
        ###################################################

        m_pc = NumpyToPCD(m_ls)
        m_pc.paint_uniform_color([1, 1, 0])
        first_pc = NumpyToPCD(first_pts_ls)
        first_pc.paint_uniform_color([1, 0,1])
        last_pc = NumpyToPCD(last_pts_ls)
        last_pc.paint_uniform_color([1, 0, 0])
        o3d.visualization.draw_geometries([first_pc, last_pc,m_pc])
        points = np.concatenate([first_pts_ls,last_pts_ls], axis=0)
        lines= [[i,i+len(last_pts_ls)] for i in range(len(last_pts_ls))]
        colors = [[1,0,0] for i in range(len(lines))]

        line_set = o3d.geometry.LineSet()
        line_set.points = o3d.utility.Vector3dVector(points)
        line_set.lines = o3d.utility.Vector2iVector(lines)
        line_set.colors = o3d.utility.Vector3dVector(colors)
        o3d.visualization.draw_geometries([line_set])
        return m_ls, first_pts_ls, last_pts_ls, colors_out#  , line_set


    def find_axis_length(self, filename, length):
        '''
        input: filename .stp CAD file that needs to be analyzed [string]
                length of the axis that need to be extracted, datum axis in STEP file hase always
                the same length, but can also include fastener axis [float]
        output: list of middle, first and last point of each axis  [np.array(N,3)]
        '''

        shape = STEP_open(filename)
        edges = get_edges(shape)
        assi = []
        length_ls = []
        for e in edges:
            l = edge_length(e)
            print(l)
            length_ls.append(l)
            if l>(length-0.0001) and l<(length+0.0001):
                assi.append(e)
        #plt.figure()
        #result = plt.hist(length_ls, bins=1000, color='c', edgecolor='k', alpha=0.65)
        #plt.show()
        first_pts_ls = []
        last_pts_ls = []
        for a in assi:
            first = topexp_FirstVertex(a)
            first_pts = [BRep_Tool().Pnt(first).Coord(1),BRep_Tool().Pnt(first).Coord(2),BRep_Tool().Pnt(first).Coord(3)]
            first_pts_ls.append(first_pts)
            last = topexp_LastVertex(a)
            last_pts = [BRep_Tool().Pnt(last).Coord(1),BRep_Tool().Pnt(last).Coord(2),BRep_Tool().Pnt(last).Coord(3)]
            last_pts_ls.append(last_pts)
        first_pts_ls = np.array(first_pts_ls)
        last_pts_ls = np.array(last_pts_ls)
        first_pc = NumpyToPCD(first_pts_ls)
        last_pc = NumpyToPCD(last_pts_ls)
        last_pc.paint_uniform_color([1, 0, 0])
        m_ls= (first_pts_ls + last_pts_ls)/2
        m_pc = NumpyToPCD(m_ls)
        m_pc.paint_uniform_color([1, 1, 0])
        m_ls= (first_pts_ls + last_pts_ls)/2
        m_ls = np.array(m_ls)
        # controllo i punti uguali
        ml_uniq,index = np.unique(m_ls,return_index = True,axis=0)
        m_ls = m_ls[index]
        first_pts_ls = first_pts_ls[index]
        last_pts_ls =last_pts_ls[index]

        #visualization
        m_pc = NumpyToPCD(m_ls)
        m_pc.paint_uniform_color([1, 1, 0])
        first_pc = NumpyToPCD(first_pts_ls)
        first_pc.paint_uniform_color([1, 0,1])
        last_pc = NumpyToPCD(last_pts_ls)
        last_pc.paint_uniform_color([1, 0, 0])
       # o3d.visualization.draw_geometries([first_pc, last_pc,m_pc])
        ###################################################
        #### check if Z direction is always the same ######
        ###################################################
        tree = KDTree(np.squeeze(m_ls))
        for i in range(len(m_ls)):
            p = m_ls[i]
            ax_p = last_pts_ls[i]-p
            indexes  = tree.query(np.reshape(p,(1,3)), k=3)[1]
            nn_mid = m_ls[indexes]
            nn_last = last_pts_ls[indexes]
            ax_vects = nn_last - nn_mid
            avg_dir = np.sum(np.squeeze(ax_vects),axis=0)/len(ax_vects)
            thrs = 0
            if ax_p@avg_dir<thrs:
                temp = copy.deepcopy(first_pts_ls[i])
                first_pts_ls[i] = last_pts_ls[i]
                last_pts_ls[i] = temp
        ###################################################

        m_pc = NumpyToPCD(m_ls)
        m_pc.paint_uniform_color([1, 1, 0])
        first_pc = NumpyToPCD(first_pts_ls)
        first_pc.paint_uniform_color([1, 0,1])
        last_pc = NumpyToPCD(last_pts_ls)
        last_pc.paint_uniform_color([1, 0, 0])
        o3d.visualization.draw_geometries([first_pc, last_pc,m_pc])
        return m_ls, first_pts_ls, last_pts_ls

    def Cluster_and_OrderPoints(self,m_ls,last_pts_ls,first_pts_ls,colors,forProfi=True,newAlg=True):
        '''
        input: mid,last,first list of point to be clustered and ordered along the main pc direction, 
                if forProfi=True add points at the tail of the cluster to extend the scanned part
        output: mid,last,first points grouped in clusters m_ls = [m_cluster1, m_cluster2..],
                the main direction of each cluster vectros = [v1, v2 ..],
                list of label for each cluster length is total numper of points cluster_labels = [1,1,1,1 ... 4,4,4 ... n,n,n,n] length of the total numper of points

        '''
        #clusterization
        #use still the old one for the test MockUp step file, it has small number of points and work better
        if newAlg ==True:
            cluster_mid, cluster_last, cluster_first, leftovers, colors  = self.Clustering_Points_new(m_ls,last_pts_ls,first_pts_ls,colors)   #, leftovers
        else:
            cluster_mid, cluster_last, cluster_first  = self.Clustering_Points(m_ls,last_pts_ls,first_pts_ls)   #, leftovers
        
        #leftovers stores points that have not been clustered

        lines = []
        colors_temp = []
        for l in cluster_mid:
            r = random.random()
            g = random.random()
            b = random.random()
            color1 = np.zeros((len(l), 3))
            color1[:, 0] = r
            color1[:, 1] = g
            color1[:, 2] = b
            colors_temp.append(color1)
            lines.append(l)
        lines = np.concatenate(lines, axis=0)
        colors_temp = np.concatenate(colors_temp, axis=0)
        pcd1 = o3d.geometry.PointCloud()
        pcd1.points = o3d.utility.Vector3dVector(lines)
        pcd1.colors = o3d.utility.Vector3dVector(colors_temp)
        o3d.visualization.draw_geometries([pcd1])
       # o3d.visualization.draw_geometries([NumpyToPCD(leftovers)])


        vectors = []
        clusters_labels = []
        for i in range(len(cluster_mid)):
            X1 = np.asarray(cluster_mid[i])
            cluster_mid[i] = np.asarray(cluster_mid[i])
            cluster_first[i] = np.asarray(cluster_first[i])
            cluster_last[i] = np.asarray(cluster_last[i])
            colors[i] = np.asarray(colors[i])
            ml_uniq,index = np.unique(cluster_mid[i],return_index = True,axis=0)
            cluster_mid[i] = cluster_mid[i][index]
            cluster_first[i] = cluster_first[i][index]
            cluster_last[i] = cluster_last[i][index]
            colors[i] = colors[i][index]
            print(str(len(cluster_mid[i])) + "--" +str(len(index)))
            pca=PCA(n_components=1)
            pca.fit(X1)
            vector = pca.components_[0]
            vectors.append(vector)
            lab_ord = self.Order_points_cluster(cluster_mid[i], vector)
            cluster_mid[i] = cluster_mid[i][lab_ord]
            cluster_last[i] = cluster_last[i][lab_ord]
            cluster_first[i] = cluster_first[i][lab_ord]
            colors[i] = colors[i][lab_ord]
                            ########  if cluster is too long divide it in chunks
            chunk = True
            if len(cluster_mid[i])>30 and len(cluster_mid[i])<=50 :
                a = math.ceil(len(cluster_mid[i])/2)
                n_points = a
            # elif len(cluster_mid[i])>40 and len(cluster_mid[i])<=60 :
            #     a = math.ceil(len(cluster_mid[i])/2)
            #     n_points = a
            # elif len(cluster_mid[i])>60 and len(cluster_mid[i])<=80 :
            #     a = math.ceil(len(cluster_mid[i])/3)
            #     n_points = a
            elif len(cluster_mid[i])>50 and len(cluster_mid[i]>=80):
                a = math.ceil(len(cluster_mid[i])/3)
                n_points = a
            elif len(cluster_mid[i])>80:
                a = math.ceil(len(cluster_mid[i])/4)
                n_points = a
            else:
                chunk = False
            if chunk == True:
                chunks_i= self.chunks(cluster_mid[i],n_points)
                cluster_mid[i] = []
                cluster_mid[i] = chunks_i

                chunks_i= self.chunks(cluster_first[i],n_points)
                cluster_first[i] = []
                cluster_first[i] = chunks_i

                chunks_i= self.chunks(cluster_last[i],n_points)
                cluster_last[i] = []
                cluster_last[i] = chunks_i

                chunks_i= self.chunks(colors[i],n_points)
                colors[i] = []
                colors[i] = chunks_i

            else:
                cluster_mid[i] = [cluster_mid[i]]
                cluster_first[i] = [cluster_first[i]]
                cluster_last[i] = [cluster_last[i]]
                colors[i] = [colors[i]]
        print("_______________________")
        temp1 = []
        temp2 = []
        temp3 = []
        temp4 = []
        v_temp = []
        for i in range(len(cluster_mid)):
            if len(cluster_mid[i])>1:
                for j in range(len(cluster_mid[i])):
                    temp1.append(np.squeeze(cluster_mid[i][j]))
                    temp2.append(np.squeeze(cluster_first[i][j]))
                    temp3.append(np.squeeze(cluster_last[i][j]))
                    temp4.append(np.squeeze(colors[i][j]))
                    v_temp.append(vectors[i])
                    

            else:
                temp1.append(np.squeeze(cluster_mid[i]))
                temp2.append(np.squeeze(cluster_first[i]))
                temp3.append(np.squeeze(cluster_last[i]))
                temp4.append(np.squeeze(colors[i]))

                v_temp.append(vectors[i])
            
        print("_______________________")
        cluster_mid = temp1
        cluster_first = temp2
        cluster_last = temp3
        colors = temp4
        vectors = v_temp
        for i in range(len(cluster_mid)):
            ml_uniq,index = np.unique(cluster_mid[i],return_index = True,axis=0)
            cluster_mid[i] = cluster_mid[i][index]
            cluster_first[i] = cluster_first[i][index]
            cluster_last[i] = cluster_last[i][index]
            colors[i] = colors[i][index]
            print(str(len(cluster_mid[i])) + "--" +str(len(index)))
        
        for i in range(len(cluster_mid)):
            for j in range(len(cluster_mid[i])):
                    #print(len(cluster_mid[i]))
                    clusters_labels.append(int(i))
        print("_______________________")
        cluster_mid_profi = copy.deepcopy(cluster_mid) 
        cluster_first_profi = copy.deepcopy(cluster_first) 
        cluster_last_profi = copy.deepcopy(cluster_last) 
        #colors_profi = copy.deepcopy(colors)
        clusters_labels_profi = []
        for i in range(len(cluster_mid_profi)):
            # if forProfi is True: 
                    #aggiungo punti agli estremi
                    #mid
            if newAlg==True:
                #DOWNSAMPLE THE CLUSTER IN ORDER TO HAVE A LINEAR SHAPE
                samples  = self.CurveFit(cluster_mid_profi[i])
                ids = self.FindIndexSamples(samples, cluster_mid_profi[i])
                #o3d.visualization.draw_geometries([NumpyToPCD(samples).paint_uniform_color([1,0,0]) , NumpyToPCD(cluster_mid_profi[i]).paint_uniform_color([0,0,1])])

                cluster_mid_profi[i] = cluster_mid_profi[i][ids]
                cluster_first_profi[i] = cluster_first_profi[i][ids]
                cluster_last_profi[i] = cluster_last_profi[i][ids]
            
                #colors_profi[i] = colors_profi[i][ids]
            #o3d.visualization.draw_geometries([NumpyToPCD(cluster_mid[i]).paint_uniform_color([1,0,0]),NumpyToPCD(cluster_mid[i]).paint_uniform_color([0,1,0]),NumpyToPCD(cluster_mid[i]).paint_uniform_color([0,0,1])])
            dist = 50
            out = self.add_tails_to_cluster(cluster_mid_profi[i], dist,vectors[i])
            cluster_mid_profi = list(cluster_mid_profi)
            cluster_mid_profi[i] = []
            cluster_mid_profi[i] = out

            #first
            out = self.add_tails_to_cluster(cluster_first_profi[i], dist,vectors[i])
            cluster_first_profi = list(cluster_first_profi)
            cluster_first_profi[i] = []
            cluster_first_profi[i] = out
            #last
            out = self.add_tails_to_cluster(cluster_last_profi[i], dist,vectors[i])
            cluster_last_profi= list(cluster_last_profi)
            cluster_last_profi[i] = []
            cluster_last_profi[i] = out
        for i in range(len(cluster_mid_profi)):
            ml_uniq,index = np.unique(cluster_mid_profi[i],return_index = True,axis=0)
            print(str(len(cluster_mid_profi[i])) + "--" +str(len(index)))
            cluster_mid_profi[i] = cluster_mid_profi[i][index]
            cluster_first_profi[i] = cluster_first_profi[i][index]
            cluster_last_profi[i] = cluster_last_profi[i][index]

                ## print("ok")
        for i in range(len(cluster_mid_profi)):
            for j in range(len(cluster_mid_profi[i])):
                #print(len(cluster_mid[i]))
                clusters_labels_profi.append(int(i))

        #######
        ##### VISUALIZE NEW FINAL CLUSTERS
        #########
        lines = []
        colors_temp = []
        for l in cluster_mid:
            r = random.random()
            g = random.random()
            b = random.random()
            color1 = np.zeros((len(l), 3))
            color1[:, 0] = r
            color1[:, 1] = g
            color1[:, 2] = b
            colors_temp.append(color1)
            lines.append(l)
        lines = np.concatenate(lines, axis=0)
        colors_temp = np.concatenate(colors_temp, axis=0)
        pcd1 = o3d.geometry.PointCloud()
        pcd1.points = o3d.utility.Vector3dVector(lines)
        pcd1.colors = o3d.utility.Vector3dVector(colors_temp)
        o3d.visualization.draw_geometries([pcd1])
       

        print("ok")
        return [cluster_mid, cluster_last, cluster_first, vectors, clusters_labels, colors], [cluster_mid_profi,cluster_first_profi,cluster_last_profi, vectors, clusters_labels_profi]
    
    def chunks(self,list,n):
        n = max(1,n)
        temp = [list[i:i+n] for i in range(0,len(list),n)]
        return temp

    def TCP_pose_from_axis_mockup(self, cluster_mid, cluster_last, cluster_first, vectors,clusters_labels, filepath='', forProfi = False, SaveFile = True ,offset = 0):
        '''
        input: middle, first, and last point of the drilling axis in clusters, list of clusters [cluster1, cluster2 ... clusterN],
                vector: PCA vector for each cluster list of np.array(3,1), 
                cluster_labels: list of  label that identify each cluster, 
                forProfi: if True offset the pose by 600mm in Z direction
                SaveFile: save the data in a .txt
                

        output: TCP position [x,y,z] and orientation[q1,q2,q3,q4] of all the point as a single list (no more clustered)
                cluster_labels, list of id number to keep track of the clusterization (can be changed)
        '''
            
        RT_ls = []
        Eul_ls = []
        Quat_ls = []
        Quat_ls2 = []
        #compute rotation matrixs
        for i in range(len(cluster_mid)):
            Quat,RT,Eul,Quat2,RT2,Eul2 = rotation_from_3points(cluster_first[i],cluster_last[i],vectors[i])
            RT = np.asarray(RT)
            Quat = np.asarray(Quat)
            Quat2 = np.asarray(Quat2)
            Eul = np.asarray(Eul)
            RT_ls.append(RT)
            Quat_ls.append(Quat)
            Eul_ls.append(Eul)
            Quat_ls2.append(Quat2)


        m_ls = np.asarray(cluster_mid)
        m_ls = np.concatenate(m_ls,axis=0) #concatenate all the clustered point in a single list, remove it if you want to keep the clusterization
        m_ls =  np.squeeze(m_ls)

        first_pts_ls = np.asarray(cluster_first)
        first_pts_ls = np.concatenate(first_pts_ls,axis=0)
        first_pts_ls = np.squeeze(first_pts_ls)

        last_pts_ls = np.asarray(cluster_last)
        last_pts_ls = np.concatenate(last_pts_ls,axis=0)
        last_pts_ls =  np.squeeze(last_pts_ls)
        RT_ls = np.asarray(RT_ls)
        RT_ls = np.concatenate(RT_ls,axis=0)
        RT_ls = np.asarray(RT_ls)

        Eul_ls = np.asarray(Eul_ls)
        Eul_ls = np.concatenate(Eul_ls,axis=0)
        Eul_ls = np.asarray(Eul_ls)

        Quat_ls = np.asarray(Quat_ls)
        Quat_ls = np.concatenate(Quat_ls,axis=0)
        Quat_ls = np.asarray(Quat_ls)

        Quat_ls2 = np.asarray(Quat_ls2)
        Quat_ls2 = np.concatenate(Quat_ls2,axis=0)
        Quat_ls2 = np.asarray(Quat_ls2)

        ########################################################
        ######### VISUALIZATION OF POSES FRAME #################
        ########################################################

        #Check if the direction is correct
        z = [0,0,1]
        v1 = RT_ls@z
        norm = np.reshape(np.linalg.norm(v1,axis=1),(len(v1),1))
        v1= v1/norm
        y = [0,1,0]
        v2 = RT_ls@y
        norm = np.reshape(np.linalg.norm(v2,axis=1),(len(v2),1))
        v2= v2/norm
        x = [1,0,0]
        #v3 = np.matmul(RT_ls,x)
        v3 = RT_ls@x
        norm = np.reshape(np.linalg.norm(v3,axis=1),(len(v3),1))
        v3= v3/norm
        if forProfi:
            off_pts1 = offset_along_vector(m_ls, -v1, offset)
            off_pts2 = copy.deepcopy(off_pts1)
            off_pts2[:,0] = off_pts2[:,0] +1
            off_pts2[:,1] = off_pts2[:,1] +1
            off_pts = [off_pts1,off_pts2]
            off_pts= np.concatenate(off_pts, axis = 0)
            Quat_ls1 = copy.deepcopy(Quat_ls)
            Quat_ls = [Quat_ls1, Quat_ls2]
            Quat_ls = np.concatenate(Quat_ls, axis= 0)
            clusters_labels1 = copy.deepcopy(clusters_labels)
            clusters_labels2 = copy.deepcopy(clusters_labels) + 1000
            clusters_labels = np.concatenate([clusters_labels1, clusters_labels2],axis = 0)
            
            #off_pts = off_pts1
            
            # # # # # ######### addded for 3d camera
            # # # temp = 0 
            # # # group=[]
            # # # group_q = []
            # # # new_pts = []
            # # # new_quats = []
            # # # new_labs = []
            # # # for i in range(len(clusters_labels)):
            # # #     if clusters_labels[i] ==temp:
            # # #         group.append(off_pts[i])
            # # #         group_q.append(Quat_ls[i])
            # # #     else:
            # # #         group = np.asarray(group)
            # # #         group_q = np.asarray(group_q)

            # # #         tx  = np.mean(group[:,0])
            # # #         ty = np.mean(group[:,1])
            # # #         tz = np.mean(group[:,2])
            # # #         tq1 = np.mean(group_q[:,0])
            # # #         tq2 = np.mean(group_q[:,1])
            # # #         tq3 = np.mean(group_q[:,2])
            # # #         tq4 = np.mean(group_q[:,3])

            # # #         new_pts.append([tx,ty,tz])
            # # #         new_quats.append([tq1,tq2,tq3,tq4])
            # # #         new_labs.append(clusters_labels[i-1])
            # # #         group = []
            # # #         group_q = []
            # # #         temp+=1
            # # # off_pts = np.asarray(new_pts)
            # # # Quat_ls = np.asarray(new_quats)
            # # # clusters_labels = np.asarray(new_labs)
            # # # #################################################



        else:
            off_pts1 =m_ls
            off_pts =m_ls
        VISUALIZE = True  
        draw = []    
        draw.append(NumpyToPCD(off_pts1).paint_uniform_color([0,0,0]))                      # allow visualization of the TCP reference frame to check if the computed orientation are correct
        for distance in range(1,10,1):
            px = offset_along_vector(off_pts1[:],v3[:],distance)
            draw.append(NumpyToPCD(px).paint_uniform_color([1,0,0]))
        for distance in range(1,10,1):
            py = offset_along_vector(off_pts1[:],v2[:],distance)
            draw.append(NumpyToPCD(py).paint_uniform_color([0,1,0]))
        for distance in range(1,10,1):
            pz = offset_along_vector(off_pts1[:],v1[:],distance)
            draw.append(NumpyToPCD(pz).paint_uniform_color([0,0,1]))
        o3d.visualization.draw_geometries(draw)
        # VISUALIZE = True                            # allow visualization of the TCP reference frame to check if the computed orientation are correct
        # if VISUALIZE:
        #     ax = plt.figure().add_subplot(projection='3d')
        #     a =0
        #     b=2000
        #     ax.scatter3D(m_ls[:,0], m_ls[:,1], m_ls[:,2],color='b')
        #     ax.scatter3D(first_pts_ls[:,0],first_pts_ls[:,1], first_pts_ls[:,2],color='g')
        #     ax.scatter3D(last_pts_ls[:,0],last_pts_ls[:,1], last_pts_ls[:,2],color='r')
        #     ax.scatter3D(off_pts[:,0],off_pts[:,1], off_pts[:,2])

        #     for distance in range(1,10,1):
        #         p = offset_along_vector(off_pts[:],v1[:],distance)
        #         ax.scatter3D(p[:,0],p[:,1], p[:,2],s=0.5,color='b')
        #     for distance in range(1,10,1):
        #         p = offset_along_vector(off_pts[:],v2[:],distance)
        #         ax.scatter3D(p[:,0],p[:,1], p[:,2],s=0.5,color='g')    
        #         print(distance)
        #     for distance in range(1,10,1):
        #         p = offset_along_vector(off_pts[:],v3[:],distance)
        #         ax.scatter3D(p[:,0],p[:,1], p[:,2],s=0.5,color='r')    
        #         print(distance)
        #     #ax.scatter3D(0,0,0, s=10, c='r')

        #     plt.show()    
        Eul_ls = np.degrees(Eul_ls)

        ############################################
        ###### SAVE POSES ON DISK #################
        ###########################################s
        if SaveFile:
            #f= open("punti_e_eulero"+cluster+".txt", "w")
            # f.write("id\tX[mm]\tY[mm]\tZ[mm]\tE1[°]\tE2[°]\tE3[°]\n")        # column names
            # np.savetxt(f, np.transpose([clusters_labels,off_pts[:,0],off_pts[:,1],off_pts[:,2],Eul_ls[:,0],Eul_ls[:,1],Eul_ls[:,2]]))
            # f.close()
            g= open( filepath, "w")
            #g.write("id\tX[mm]\tY[mm]\tZ[mm]\tq1\tq2\tq3\tq4\n")        # column names
            np.savetxt(g, np.transpose([np.asarray(clusters_labels).astype(float),np.asarray(off_pts[:,0]).astype(float),np.asarray(off_pts[:,1]).astype(float),np.asarray(off_pts[:,2]).astype(float),np.asarray(Quat_ls[:,0]).astype(float),np.asarray(Quat_ls[:,1]).astype(float),np.asarray(Quat_ls[:,2]).astype(float),np.asarray(Quat_ls[:,3]).astype(float)]))
            g.close()
          

        return [off_pts, Quat_ls, clusters_labels]
    
    def Order_points_cluster(self,cluster_pts, PCA1):
        '''
        input: list of points np.array(N,3), PCA1 main direction of the points
        output: ordered indexes of the point list jd = list of indexes
        '''
        #main direction of cluster between x,y,z
        id = np.argmax(np.abs(PCA1))
        # find extremes of the clusters
        start_id = np.argmin(cluster_pts[:,id])
        start_pt = cluster_pts[start_id]
        # compute distances in main PCA direction from start point
        delta_ls = []
        for i in range(len(cluster_pts)):
            delta_ls.append(abs(start_pt[id] - cluster_pts[i,id]))
        #reorder cluster 
        jd = np.argsort(np.asarray(delta_ls))

        #cluster_pts_ordered  = cluster_pts[jd]

        return  jd

    def add_tails_to_cluster(self,cluster, dist,vector):
        '''
        input: clster point list, distance to add an offset point
        output: cluster point list with added point on top and bottom
        '''
        #o3d.visualization.draw_geometries([NumpyToPCD(cluster)])
        i = 0
        # ml_uniq,index = np.unique(cluster,return_index = True,axis=0)
        # print(str(len(cluster)) + "--" +str(len(index)))
        # ml_uniq,index = np.unique(cluster,return_index = True,axis=0)
        # print(str(len(cluster)) + "--" +str(len(index)))
        # cluster = cluster[index]

        try:
            while True:
                if  cluster[0][0] == cluster[i][0] and cluster[0][1] == cluster[i][1] and cluster[0][2] == cluster[i][2]:
                    i=i+1
                    if i>=len(cluster):
                        w1 = vector
                        print("dim 1")
                        break
                    # if i==len(cluster):
                    #     break
                else:
                    w1 = cluster[0]-cluster[i]     
                    break
            pt1  = offset_along_vector(cluster[0], w1, dist)
            cluster = cluster.tolist()
            cluster = [pt1] + cluster 
            cluster = np.asarray(cluster)
            j = 0
            while True:
                if  cluster[-1][0] == cluster[-1-j][0] and cluster[-1][1] == cluster[-1-j][1] and cluster[-1][2] == cluster[-1-j][2] :
                    j=j+1
                    if j>=len(cluster):
                        w1 = -vector
                        print("dim 1")
                        break
                    # if j==len(cluster):
                    #     break
                else:
                    w2 = cluster[-1]-cluster[-1-j]     
                    break
        except:
            print("error")
            
        #w2 = cluster[-1]-cluster[-2]
        pt2  = offset_along_vector(cluster[-1], w2, dist)
        cluster = cluster.tolist()
        cluster.append(pt2)
        return np.asarray(cluster)

    def Clustering_Points(self, center,normal,antinormal):
        '''
        input: points to be clustered with DB scan, middle, first and last points are passed in order to be sure the clusterization is the same
        output: list of clustered points, + 
        '''
        data_temp=center
        # #############################################################################
        model2 = cluster.DBSCAN(eps=50,min_samples=3)
        #model2 = cluster.AgglomerativeClustering(n_clusters=20)
        model2.fit_predict(data_temp)
        pred2= model2.fit_predict(data_temp)
        VISUALIZE = True
        # if VISUALIZE:
        #     fig = plt.figure()
        #     ax = Axes3D(fig)
        #     ax.scatter(data_temp[:,0], data_temp[:,1], data_temp[:,2],c = model2.labels_, s=10)
        #     ax.view_init(azim=200)
        #     plt.show()
        clusters  = [[]for n in range(max(model2.labels_)+1)]  # inizialize point clusters array
        count = 0
        clusters2  = [[]for n in range(max(model2.labels_)+1)]
        clusters3  = [[]for n in range(max(model2.labels_)+1)]

        for i in range(max(model2.labels_)+1):
            bol = np.equal(model2.labels_,np.ones((len(data_temp),))*i,)
            p= data_temp[bol]
            p2 = normal[bol]
            p3 = antinormal[bol]

            clusters[i].append(p)
            clusters2[i].append(p2)
            clusters3[i].append(p3)
            count+=1
            #print(count)
        #lim = 30
        # for j in range(len(clusters2)):
        #     c2= clusters2[j]
        #     c3= clusters3[j]
        #     for k in range(len(np.squeeze(c2))):
        #         v1= c2[0][0] - c3[0][0]
        #         v2= c2[0][k] - c3[0][k]
        #         scal = np.inner(v1,v2)
        #         if scal<lim:
        #             tmp = clusters2[j][0][k]
        #             clusters2[j][0][k]= clusters3[j][0][k]
        #             clusters2[j][0][k]=tmp

        return np.squeeze(clusters).tolist(),np.squeeze(clusters2).tolist(),np.squeeze(clusters3).tolist()

    def Clustering_Points_new(self, points,normal,antinormal,colors ,min_ratio = 0.1):   #min_ratio  = %of leftovers accepted
        o3d.visualization.draw_geometries([NumpyToPCD(points)])
        N = len(points)
        target = points.copy()
        count = 0
        model = LineModelND()
        model.estimate(target)
        center_list = []
        normal_list = []
        antinormal_list = []
        color_list = []
        while count < (1 - min_ratio) * N:
            model_robust, inliers = ransac(target, LineModelND, min_samples=4,
                                residual_threshold=math.ceil(len(target)/15), max_trials=6000)    # residual_threshold and max_trials main parameters to tune for desired clusterization outcome
            true_numb = np.count_nonzero(inliers)
            indexes = np.argwhere(inliers)
            count += true_numb
            center_list.append(np.squeeze(target[indexes]))
            normal_list.append(normal[indexes])
            antinormal_list.append(antinormal[indexes])
            color_list.append(colors[indexes])
            target = np.delete(target, indexes, axis=0)
            normal = np.delete(normal, indexes, axis=0)
            antinormal = np.delete(antinormal, indexes, axis=0)
        return center_list, normal_list, antinormal_list, target, color_list
    

    def CurveFit(self,points):
        # ####order points
        # data = copy.deepcopy(points)
        # # # if len(data)<=6:
        # # #     samples = data
        # # # else:
        # pca = PCA(n_components=3)
        # pca.fit(data)
        # main_dir = pca.components_[0]
        # jd = self.Order_points_cluster(data, main_dir)
        
        # data = data[jd]
        # a = data[0]
        # b = data[-1]
        # #check if 3rd direction is big
        # if main_dir@(b-a)<0:
        #     main_dir = -main_dir
        # test = copy.deepcopy(data)
        # main_dir2 = pca.components_[2]
        # kd = self.Order_points_cluster(test, main_dir2)
        # test = test[kd]
        # a1 = test[0]
        # b1 = test[-1]
    
        # dist = np.linalg.norm(b-a)
        # norm = np.linalg.norm(main_dir)
        # main_dir = main_dir/norm
    
        # # if  np.linalg.norm(b1-a1)>50:
        # samples = []
        
        # #samples.append(data[0])
        # # if len(data)>9:
        # #     n = int(len(data)/3)
        # # else:
        # n = len(data)
        # for i in range(n+1):
        #     off_pt = data[0]+(dist/n*i)*main_dir
        #     samples.append(off_pt)
            
        # samples = np.squeeze(np.asarray(samples))
        
        # s = samples
        # d = data

        # X = s[:,0]
        # Y = s[:,1]
        # # best-fit cubic curve
        # A = np.c_[np.ones(d.shape[0]), d[:,:2], np.prod(d[:,:2], axis=1), d[:,:2]**2, d[:,:2]**3]
        # C,_,_,_ = scipy.linalg.lstsq(A, d[:,2])
        # # evaluate it on a grid
        # Z = np.dot(np.c_[np.ones(X.shape), X, Y, X*Y, X**2, Y**2, X**3, Y**3], C).reshape(X.shape)
        # samples = np.squeeze(np.asarray([X,Y,Z])).T

                
        #     # else: 
            
        #     #     if len(data)>10:
        #     #         samples = data[0:-1:int(np.round(len(data)/10))]
                   

        #     #     else:
        #     #         samples = data
        x = points[:,0]
        y = points[:,1]
        z = points[:,2]
        max = 0
        min = 0
        a = np.max(z)
        b = np.min(z)
        # ### find extreme of points
        # for i in range(len(points)):
        #     pt = points[i]
        #     x = pt[0]
        #     y = pt[1]
        #     if x+y > max:
        #         max = x+y
        #         id_max = i
        #     if x+y <
        A_xz = np.vstack((x, np.ones(len(x)))).T
        m_xz, c_xz = np.linalg.lstsq(A_xz, z)[0]
        A_yz = np.vstack((y, np.ones(len(y)))).T
        m_yz, c_yz = np.linalg.lstsq(A_yz, z)[0]
        zz = np.linspace(b,a)
        xx,yy = self.lin(zz,c_xz,m_xz,c_yz,m_yz)
        samples = np.asarray([xx,yy,zz]).T

       # o3d.visualization.draw_geometries([NumpyToPCD(samples).paint_uniform_color([1,0,0]),NumpyToPCD(points).paint_uniform_color([0,0,1])])
        return samples

    def FindIndexSamples(self,samples, points):
        tree = KDTree(np.asarray(points))
        distances, index = tree.query(np.asarray(samples))
        return index

    def lin(self,z,c_xz,m_xz,c_yz,m_yz):
        x = (z - c_xz)/m_xz
        y = (z - c_yz)/m_yz
        return x,y