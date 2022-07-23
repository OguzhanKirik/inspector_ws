#! /usr/bin/env python3
from posixpath import expanduser
#from camera_3d.src.camera3d_node import grab_3d_pointcloud
import rospy
import sys
from os.path import dirname, abspath
import os
# include path python
include_dir = str(dirname(dirname(abspath(__file__)))) + '/include/'
sys.path.append(include_dir)
from pathlib import Path
utils_dir = str(Path(__file__).resolve().parent.parent.parent)
sys.path.append(utils_dir)
import pyforms
from pyforms.basewidget import BaseWidget
from pyforms.controls   import ControlFile
from pyforms.controls   import ControlText
from pyforms.controls   import ControlSlider
from pyforms.controls   import ControlPlayer
from pyforms.controls   import ControlButton
from pyforms.controls   import ControlCheckBox
from pyforms.controls   import ControlNumber
from pyforms.controls   import ControlImage
from pyforms.controls   import ControlProgress
from pyforms.controls   import ControlCombo
from frank.msg import baseStruct, structArray
import rospy
import numpy as np
import cv2
from std_msgs.msg import Bool, Int32, Int16
from sensor_msgs.msg import Image, PointCloud2
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint
from UTILS.cv_bridge_my import *
from ros_numpy.point_cloud2 import pointcloud2_to_xyz_array
import random
import open3d as o3d
from UTILS.pcd_numpy_utils import *
from frank.srv import *
from frank.msg import *

from geometry_msgs.msg import Twist, Transform
import threading
import time
import json

class FrankCommander(BaseWidget):
    cam2d_connected = False
    cam3d_connected = False
    profi_connected = False
    response_acquisition = None
    robot_connected = False
    tool_mounted = 0
    egm_mode = 0
    PLC_connected = False
    count=0
    num_img= 0 
    rospy.init_node("GUI_master_node")
    def __init__(self):
      
      super(FrankCommander,self).__init__('Frank Commander')
      self.StartSparseScan = ControlButton('Start Sparse Scan')
      self.StartSparseScan.value = self.buttonStartSparseScan
      #camera 2d
      self.take_2d_snapshot = ControlButton('Take 2D Snapshot')
      self.take_2d_snapshot.value = self.buttonTake2DPhoto
      self.connect2D = ControlButton('Connect 2D Camera')
      self.connect2D.value = self.buttonConnect2DCamera
      self.disconnect2D = ControlButton('Disconnect 2D Camera')
      self.disconnect2D.value = self.buttonDisconnect2DCamera
      #camera_3d
      self.take_3d_snapshot = ControlButton('Take 3D Snapshot')
      self.take_3d_snapshot.value = self.buttonTake3DSnapshot
      self.connect3D = ControlButton('Connect 3D Camera')
      self.connect3D.value = self.buttonConnect3DCamera
      self.disconnect3D = ControlButton('Disconnect 3D Camera')
      self.disconnect3D.value = self.buttonDisconnect3DCamera
      self.densescan3D = ControlButton('Dense_Scan Camera')
      self.densescan3D.value = self.buttonDenseScan3DCamera
      #profilometer
      self.start_acquisition= ControlButton('StartScan')
      self.start_acquisition.value = self.buttonStartProfi
      self.stop_acquisition= ControlButton('StopScan')
      self.stop_acquisition.value = self.ButtonStopProfi
      self.connectProfi = ControlButton('Connect Profi')
      self.connectProfi.value = self.buttonConnectProfilometer
      self.disconnectProfi = ControlButton('Disconnect Profi')
      self.disconnectProfi.value = self.buttonDisconnectProfilometer
      #robot
      self.connect_robot = ControlButton('Connect_Robot')
      self.connect_robot.value = self.buttonConnectRobot
      self.set_tool = ControlButton('Set_Tool')
      self.set_tool.value = self.buttonSetTool
      self.tool_value = ControlText("Tool Choice [0,1,2]", default = "0")
      self.set_egm_mode = ControlButton('Set_EGM_mode')
      self.set_egm_mode.value = self.buttonSetEGMMode
      self.egm_value = ControlText("EGM mode [0 (Pose), 1(Joint)]", default = "1")
      #self.disconnect_robot.value = self.buttonDisonnectRobot # command it 
      self.go_to_pos = ControlButton('Go_To_Target')
      self.go_to_pos.value = self.buttonGoToPos
      self.target_value = ControlText("Target [x y z q1 q2 q3 q4]", default = "900 25 765.56 0 -0.707106781 0.707106781 0")
      #PLC
      self.connect_plc = ControlButton("Connect PLC")
      self.connect_plc.value = self.buttonConnectPLC
      self.disconnect_plc = ControlButton("Disconnect PLC")
      self.disconnect_plc.value = self.buttonDisconnectPLC
      self.set_manual_plc = ControlButton("SET MANUAL PLC")
      self.set_manual_plc.value = self.SetManualPLC
      self.set_automatic_plc = ControlButton("SET AUTOMATIC PLC")
      self.set_automatic_plc.value = self.SetAutomaticPLC
      self.set_zero_plc = ControlButton("SET ZERO PLC")
      self.set_zero_plc.value = self.SendZeroPLC
      self.state_command_value = ControlText("State Request : ", default = "3")
      self.state_command_req = ControlButton("SEND STATE REQUEST")
      self.state_command_req.value = self.CallStateRequest
      self.reset_plc = ControlButton("RESET PLC")
      self.reset_plc.value = self.SendPLCReset
      self.send_sparse = ControlButton("SEND SPARSE")
      self.send_sparse.value = self.buttonSendSparseScan
      self.set_drill_speed = ControlButton("START DRILL")
      self.set_drill_speed.value = self.buttonStartDrill
      self.drill_speed_value = ControlText("[°/sec] : ", default = "72000")
      self.stop_drill = ControlButton("STOP DRILL")
      self.stop_drill.value = self.buttonStopDrill

      self.choose_procedure = ControlButton("START PROCEDURE")
      self.choose_procedure.value = self.buttonChooseProcedure
      self.sparse_mode_value = ControlText("Align mod [ 1 auto , 0 manual] ", default = "1")
      self.sparse_align = ControlButton("SPARSE ALIGNMENT")
      self.sparse_align.value = self.buttonSparseAlignment
      self.dense_align = ControlButton("DENSE ALIGNMENT")
      self.dense_align.value = self.buttonDenseAlignment
      self.dense_cluster_value = ControlText("cluster id", default = "0")
      self.axis_clustering = ControlButton("AXIS CLUSTERING")
      self.axis_clustering.value = self.buttonAxisClustering
      self.axis_colors = ControlText("colors[[r,g,b]]", default = "[[1,0,0],[1,1,0]]")

      self.procedure_value = ControlText("Procedure num : ", default = "-5")
      self.drill_value = ControlText("DrillBit : ", default = "3")
      self.set_current_tool = ControlButton("Set Current Tool")
      self.set_current_tool.value = self.buttonSetCurrentTool
      self.current_tool_value = ControlText("current drillbit: ", default = "0")

      self.cluster_to_scan = ControlText("clusters [1,2,..]", default = "[1,12,13,1000,1012,1013]")

      self.formset = [ {
      'a:Master Controller':
                  [
                    #('StartSparseScan'),
                    ('connect2D','take_2d_snapshot','disconnect2D'),
                    ('connect3D','take_3d_snapshot','disconnect3D'),
                    ('connectProfi','disconnectProfi'),
                    #('connect_robot','tool_value','set_tool'),
                    #('set_egm_mode','egm_value'),
                    ('go_to_pos', 'target_value'),
                    ('connect_plc','disconnect_plc','set_automatic_plc','set_manual_plc','set_zero_plc','reset_plc'),
                    ('set_drill_speed','drill_speed_value','stop_drill'),
                    ("cluster_to_scan"),
                    ('choose_procedure','procedure_value','drill_value','current_tool_value','set_current_tool'),
                    ('axis_clustering','axis_colors','sparse_align','sparse_mode_value','','','','dense_align','dense_cluster_value')
                    ]
      # 'b:Calibration':
      #             [
      #               ()
      #             ] 
                  }]

    ##############################
    #### CALLBACKS DEFINITION ####
    ##############################
    def _image_callback(self, resp,filepath,num):
      if resp.res==0:
        print(resp.error)
        return 
          #if msg.error == 1:
      if resp.res ==1:
        print(resp.error)
        print("Recived an Image!")
        cv2_img = imgmsg_to_cv2(resp.image)
        rospy.sleep(1)
        cv2.imwrite(filepath+'/camera_image'+str(num)+'.jpeg',cv2_img)
        print('saved image')
        return
          #if msg.error ==-1:
            #print('error in taking picture')

    def _2dcam_disconnection_callback(self,resp):
          if resp.res==0:
            print(resp.error)
            return
          if resp.res==1:
            print(resp.error)
            self.cam2d_connected = False
            return
          else:
            print('unexpected service response')
            return
  

    def _2dcam_connection_callback(self,resp):
          if resp.res==0:
            print(resp.error)
            self.cam2d_connected = False
            return
          elif resp.res==1:
            print(resp.error)
            self.cam2d_connected = True
            return
          else:
            print('unexpected service response')
            return
    

    def _3dcam_connection_callback(self,resp):
          if resp.res==0:
            print(resp.error)
            return
          elif resp.res==1:
            print(resp.error)
            self.cam3d_connected = True
            return
          else:
            print('unexpected service response')
            return
    def _3dcam_disconnection_callback(self,resp):
          if resp.res==0:
            print(resp.error)
            return
          if resp.res==1:
            print(resp.error)
            self.cam3d_connected = False
            return
          else:
            print('unexpected service response')
            return

    def _pointcloud_callback(self,resp,filepath,num):
      if resp.res==0:
        print(resp.error)
        return 
          #if msg.error == 1:
      if resp.res ==1:
        print(resp.error)
        print("Recived a pointcloud!")
        points = pointcloud2_to_xyz_array(resp.pointcloud)
        rospy.sleep(1)
        o3d.visualization.draw_geometries([NumpyToPCD(points)])
        
        o3d.io.write_point_cloud(filepath+'/3dCam_poincloud'+str(num)+'.ply',NumpyToPCD(points) )

        print('saved pointcloud')
        return
          #if msg.error ==-1:
            #print('error in taking picture')
      else:
        print('unexpected service response')
        return

    def _profi_connection_callback(self,resp):
          if resp.res==0:
            print(resp.error)
            return
          elif resp.res==1:
            print(resp.error)
            self.profi_connected = True
            return
          elif resp.res==2:
            print(resp.error)
            self.profi_connected = True  ##############################################################à
          else:
            print('unexpected service response')
            return

    def _profi_disconnection_callback(self,resp):
          if resp.res==0:
            print(resp.error)
            return
          if resp.res==1:
            print(resp.error)
            self.profi_connected = False
            return
          else:
            print('unexpected service response')
            return

    def AcquisitionThreadHelp(self, startprof, response):
      response = startprof()
      return 0

    def _acquisition_callback(self,resp):
      if resp.res ==0:
        print(resp.error)
        return
      if resp.res==1:
        print(resp.error)
        # profiles = resp.profiles
        # XX = profiles.XX
        # YY = profiles.YY
        # ZZ = profiles.ZZ
        # pcd = o3d.geometry.PointCloud()
        # pcd.points = o3d.utility.Vector3dVector(np.asarray([XX,YY,ZZ].T))
        # o3d.visualization.draw_geometries([pcd])
      self.response_acquisition=None
      return

    def stop_acqu_callback(self,resp):
      print(resp.error)
    ##################
    ###### Robot #####
    ##################

    def _rob_conn_callback(self,res):
      if res.res == 1:
        print(res.error)
        self.robot_connected =True
        return
      if res.res ==0:
        print(res.error)
        return
    def _set_tool_callback(self,res):
      print(res.error)
      if res.res ==1:
         self.tool_mounted = self.tool_value.value
      print(res.error)
      return
    def _egm_mod_callback(self,res):
      print(res.error)
      if res.res ==1:
         self.egm_mode = self.egm_value.value
      else:
        print("errore cambio egm")
      return

    ##################################
    ###### ACTION DEFINITIONS ########
    ##################################
    def buttonStartSparseScan(self):
        print('starting sparse scan')
        return

##################################
###### CAMERA 3D ###############
################################
    def buttonTake3DSnapshot(self):
        time.sleep(3)
        if self.cam3d_connected==False:
          print('Camera 3D is not Connected\n Please connect to the camera')
          return
        else:
          rospy.wait_for_service('grab_3dpointcloud',timeout=1)
          print('grab 3d snapshot from 3d camera')
          try:
            grab3d = rospy.ServiceProxy("grab_3dpointcloud",Grab3DPointcloud)
            resp = grab3d()
            filepath = "/home/oguz/Desktop/TEMP/CALIB_3DCAM"
            num = self.count
            self._pointcloud_callback(resp, filepath,num)
            #save coords
            getpos =  rospy.ServiceProxy("GetPose",GetPose)
            resp = getpos()
            pose = list(resp.pose[2:])
            print(pose)
            fCoord = open(filepath + "/Coords"+str(num)+".txt", "w")
            fCoord.write("Number\tX [mm]\tY [mm]\tZ [mm]\tqw\tqz\tqy\tqz\n"+
            str(num)+"\t"+str(pose[0])+"\t"+str(pose[1])+"\t"+str(pose[2])+"\t"+str(pose[3])+"\t"+str(pose[4])+"\t"+str(pose[5])+"\t"+str(pose[6]))
            fCoord.close()

            self.count+=1
            return
          except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            return
        return

    def buttonConnect3DCamera(self):
        if self.cam3d_connected == True:
          print('camera is already connected')
          return
        else:
          rospy.wait_for_service('connect_3dcamera',timeout=1)
          print('connecting to 3d camera x36')
          try:
            conn3d = rospy.ServiceProxy("connect_3dcamera",Connect3DCamera)
            resp = conn3d()
            self._3dcam_connection_callback(resp)
            return
          except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            return

    def buttonDisconnect3DCamera(self):
        if self.cam3d_connected == False:
          print('camera is already disconnected')
          return
        else:
          rospy.wait_for_service('disconnect_3dcamerax36',timeout=1)
          print('disconnecting to 3d camera')
          try:
            disconn3d = rospy.ServiceProxy("disconnect_3dcamerax36",Disconnect3DCamerax36)
            resp = disconn3d()
            self._3dcam_disconnection_callback(resp)
            return
          except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            return
    def buttonSendSparseScan(self):
        # if self.cam3d_connected == False:
        #   print('camera is already disconnected')
        #   return
        # else:
          
          try:
            rospy.wait_for_service('send_sparse_pointcloud',timeout=1)
            print('disconnecting to 3d camera')
            send_sparse = rospy.ServiceProxy("send_sparse_pointcloud",SendSparseScan)
            resp = send_sparse()
            print(resp.error)
            points = pointcloud2_to_xyz_array(resp.pointcloud)
            rospy.sleep(1)
            o3d.visualization.draw_geometries([NumpyToPCD(points)])
            return
          except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            return
#######################
####### Profilometer #####
#######################

    def buttonStartProfi(self):
        # if self.profi_connected==False:
        #   print('profilometer is not Connected\nPlease connect to the profilometer')
        #   return
        # else:
          
          try:
            rospy.wait_for_service('start_profi_acquisition',timeout=1)
            print('starting acquisition')
            startprof = rospy.ServiceProxy("start_profi_acquisition",StartProfi)
            resp = startprof()
            #t1 = threading.Thread(target = self.AcquisitionThreadHelp, args = (startprof, self.response_acquisition))
            #t1.start()
            # while self.response_acquisition==None:
            #   time.sleep()
            # t1.join()
            self._acquisition_callback(resp)
            return
          except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            return
          return
    def ButtonStopProfi(self):
      # if self.profi_connected==False:
      #     print('profilometer is not Connected\nPlease connect to the profilometer')
      #     return
      # else:
       
        try:
          rospy.wait_for_service("stop_profi_acquisition",timeout=1)
          print('Stopping Acquisition')
          stopprof = rospy.ServiceProxy("stop_profi_acquisition", StopProfi)
          resp = stopprof()
          self.stop_acqu_callback(resp)
          return

        except rospy.ServiceException as e:
          print("Service call failed: %s"%e)
          return

    def buttonConnectProfilometer(self):
        # if self.profi_connected == True:
        #   print('profilometer is already connected')
        #   return
        # else:
          
          try:
            rospy.wait_for_service('connect_profilometer',timeout=1)
            print('connecting to profilometer')
            connprof = rospy.ServiceProxy('connect_profilometer',ConnectProfilometer)
            resp = connprof()
            self._profi_connection_callback(resp)
            return
          except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            return

    def buttonDisconnectProfilometer(self):
          # if self.profi_connected == False:
          #   print('profilometer is already disconnected')
          #   return
          # else:
      
            try:
              rospy.wait_for_service('disconnect_profilometer',timeout=1)
              print('disconnecting from profilometer')
              disconnprof = rospy.ServiceProxy("disconnect_profilometer",DisconnectProfilometer)
              resp = disconnprof()
              self._profi_disconnection_callback(resp)
              return
            except rospy.ServiceException as e:
              print("Service call failed: %s"%e)
              return




#######################
####### CAMERA 2D #####
#######################

    def buttonTake2DPhoto(self):
        # if self.cam2d_connected==False:
        #   print('Camera 2D is not Connected\nPlease connect to the camera')
        #   return
        # else:
          
          try:
            filepath = "/home/oguz/Desktop/TEMP/2DCAM"
            rospy.wait_for_service('grab_2dimage',timeout=1)
            print('grabbing 2d image')
            grab2d = rospy.ServiceProxy("grab_2dimage",Grab2DImage)
            resp = grab2d()
            self._image_callback(resp,filepath,self.num_img)
            self.num_img+=1
            # getpos =  rospy.ServiceProxy("GetPose",GetPose)
            # resp = getpos()
            # pose = list(resp.pose[2:])
            # print(pose)
            # fCoord = open(filepath + "/Coords"+str(self.num_img)+".txt", "w")
            # fCoord.write("Number\tX [mm]\tY [mm]\tZ [mm]\tqw\tqz\tqy\tqz\n"+
            # str(self.num_img)+"\t"+str(pose[0])+"\t"+str(pose[1])+"\t"+str(pose[2])+"\t"+str(pose[3])+"\t"+str(pose[4])+"\t"+str(pose[5])+"\t"+str(pose[6]))
            # fCoord.close()
            return
          except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            return
        # return

    def buttonConnect2DCamera(self):
        # if self.cam2d_connected == True:
        #   print('camera is already connected')
        #   return
        # else:
          
          try:
            rospy.wait_for_service('connect_2dcamera',timeout=1)
            print('connecting to 2d camera')
            conn2d = rospy.ServiceProxy("connect_2dcamera",Connect2DCamera)
            resp = conn2d()
            self._2dcam_connection_callback(resp)
            return
          except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            return

    def buttonDisconnect2DCamera(self):
          # if self.cam2d_connected == False:
          #   print('camera is already disconnected')
          #   return
          # else:
            
            try:
              rospy.wait_for_service('disconnect_2dcamera',timeout=1)
              print('disconnecting to 2d camera')
              disconn2d = rospy.ServiceProxy("disconnect_2dcamera",Disconnect2DCamera)
              resp = disconn2d()
              self._2dcam_disconnection_callback(resp)
              return
            except rospy.ServiceException as e:
              print("Service call failed: %s"%e)
              return


    def buttonConnectRobot(self):
      if self.robot_connected==True:
        print("robot is already Connected")
        return
      else:
        rospy.wait_for_service('Start_Robot',timeout = 1)
        print("connecting to robot")
        try:
          conn_rob = rospy.ServiceProxy("Start_Robot", Start_Robot)
          res = conn_rob()
          self._rob_conn_callback(res)
        except rospy.ServiceException as e:
              print("Service call failed: %s"%e)
              return

    def buttonSetTool(self):
      if self.robot_connected==False:
        print("robot is not Connected")
        return
      else:
        rospy.wait_for_service('Set_Tool',timeout = 1)
        print("changing tool")
        try:
          tool_rob = rospy.ServiceProxy("Set_Tool",Set_Tool)
          req = Set_ToolRequest()
          req.tool = int(self.tool_value.value)
          res = tool_rob(req)
          self._set_tool_callback(res)
        except rospy.ServiceException as e:
              print("Service call failed: %s"%e)
              return

    def buttonSetEGMMode(self):
      if self.robot_connected==False:
        print("robot is not Connected")
        return
      else:
        rospy.wait_for_service('Set_EGM_mode',timeout = 1)
        print("changing egm mode")
        try:
          s_egmmode = rospy.ServiceProxy("Set_EGM_mode",SetEGMMode)
          req = SetEGMModeRequest()
          req.mod = int(self.egm_value.value)
          res = s_egmmode(req)
          self._egm_mod_callback(res)
        except rospy.ServiceException as e:
              print("Service call failed: %s"%e)
              return
    def buttonGoToPos(self):
      # if self.robot_connected==False:
      #   print("robot is not Connected")
      #   return
      # else:
        rospy.wait_for_service("GoToPose", timeout=1)
        print("going to target position")
        try:
          s_gotopos = rospy.ServiceProxy("GoToPose", GoToPose)
          req = GoToPoseRequest()
          target_point = self.target_value.value.split(" ")
          target_point = [float(x) for x in target_point]
          print(target_point)
          # point = MultiDOFJointTrajectoryPoint()
          # point.transforms.append(Transform())
          # point.transforms[0].translation.x = target_point[0]
          # point.transforms[0].translation.y = target_point[1]
          # point.transforms[0].translation.z = target_point[2]
          # point.transforms[0].rotation.x = target_point[3]
          # point.transforms[0].rotation.y = target_point[4]
          # point.transforms[0].rotation.z = target_point[5]
          # point.transforms[0].rotation.w = target_point[6]
          # for i in range(len(target_point)):
          #   req.pose.append(target_point[i])
          req.pose = list(target_point)
          response = s_gotopos(req)
          # print(response.error)
          # print(response.res)
        except rospy.ServiceException as e:
          print("Service call failed: %s"%e)
          return  
###########################
####### PLC ##############
#########################
    def buttonConnectPLC(self):
            # if self.PLC_connected == True:
            #   print('PLC is already connected')
            #   return
            # else:
              
              try:
                rospy.wait_for_service('plc_connect',timeout=1)
                print('connecting to plc')
                conn_plc= rospy.ServiceProxy("plc_connect",ConnectPLC)
                resp = conn_plc()
                if resp.res==1:
                  self.PLC_connected =True
                print(resp.error)
                return
              except rospy.ServiceException as e:
                print("Service call failed: %s"%e)
                return

    def buttonDisconnectPLC(self):
            # if self.PLC_connected == False:
            #   print('PLC is already disconnected')
            #   return
            # else:
              
              try:
                rospy.wait_for_service('plc_disconnect',timeout=1)
                print('disconnecting to plc')
                conn_plc= rospy.ServiceProxy("plc_disconnect",DisconnectPLC)
                resp = conn_plc()
                if resp.res==1:
                  self.PLC_connected =False
                print(resp.error)
                return
              except rospy.ServiceException as e:
                print("Service call failed: %s"%e)
                return

    def SetManualPLC(self):
        # if self.PLC_connected == False:
        #       print('PLC is  disconnected')
        #       return
        # else:
          
          try:
            rospy.wait_for_service('plc_command',timeout=1)
            print('setting plc to manual')
            conn_plc= rospy.ServiceProxy("plc_command",CommandPLC)
            comando =  CommandPLCRequest()
            comando.command = "GVL_ROS.IRC5_WORD1_ST"
            comando.value = 1
            resp = conn_plc(comando)
            print(resp.error)
            return
          except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            return
    
    def SetAutomaticPLC(self):
        # if self.PLC_connected == False:
        #       print('PLC is  disconnected')
        #       return
        # else:
          
          try:
            rospy.wait_for_service('plc_command',timeout=1)
            print('setting plc to automatic')
            conn_plc= rospy.ServiceProxy("plc_command",CommandPLC)
            comando =  CommandPLCRequest()
            comando.command = "GVL_ROS.IRC5_WORD1_ST"
            comando.value = 2
            resp = conn_plc(comando)
            print(resp.error)
            return
          except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            return
    def SendZeroPLC(self):
        # if self.PLC_connected == False:
        #       print('PLC is  disconnected')
        #       return
        # else:
          
          try:
            rospy.wait_for_service('plc_command',timeout=1)
            print('SENDING ZERO PROGRAM TO PLC')
            conn_plc= rospy.ServiceProxy("plc_command",CommandPLC)
            comando =  CommandPLCRequest()
            comando.command ="GVL_ROS.IRC5_ParteProgrammaAct"
            comando.value = 0
            resp = conn_plc(comando)
            print(resp.error)
            return
          except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            return
    def SendPLCReset(self):
        # if self.PLC_connected == False:
        #       print('PLC is  disconnected')
        #       return
        # else:
         
          try:
            rospy.wait_for_service('plc_command',timeout=1)
            print('SENDING RESET TO PLC')
            conn_plc= rospy.ServiceProxy("plc_command",CommandPLC)
            comando =  CommandPLCRequest()
            comando.command ="GVL_ROS.IRC5_Rst"
            comando.value = 1
            resp = conn_plc(comando)
            print(resp.error)
            time.sleep(1)
            comando.value = 0
            resp = conn_plc(comando)
            print(resp.error)
            return
          except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            return

    def CallStateRequest(self):
      # if self.PLC_connected == False:
      #         print('PLC is  disconnected')
      #         return
      # else:
          print('SENDING STATE COMMAND')
          try:
            state_rob_pub = rospy.Publisher("/state_request", Int16,queue_size=1)
            msg = Int16()
            msg.data = int(self.state_command_value.value)
            state_rob_pub.publish(msg)
            return
          except :
            print("errore sconosciuto")
            return

    def buttonStartDrill(self):
      # if self.PLC_connected == False:
      #         print('PLC is  disconnected')
      #         return
      # else:
          print('STARTING DRILLER')
          try:
            rospy.wait_for_service('plc_command',timeout=1)
            conn_plc= rospy.ServiceProxy("plc_command",CommandPLC)
            comando =  CommandPLCRequest()
            comando.command ="GVL_ROS.EM_SetpointSpeed"
            comando.value = int(self.drill_speed_value.value)
            resp = conn_plc(comando)
            print(resp.error)
    
            return
          except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            return
    def buttonStopDrill(self):
      # if self.PLC_connected == False:
      #         print('PLC is  disconnected')
      #         return
      # else:
          print('STOPPING DRILLER')
          try:
            rospy.wait_for_service('plc_command',timeout=1)
            conn_plc= rospy.ServiceProxy("plc_command",CommandPLC)
            comando =  CommandPLCRequest()
            comando.command ="GVL_ROS.EM_SetpointSpeed"
            comando.value = 0
            resp = conn_plc(comando)
            print(resp.error)
    
            return
          except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            return

    def buttonChooseProcedure(self):

        procedure = int(self.procedure_value.value)
        drillbit = int(self.drill_value.value)
        TocheckStruct=[]
        StructChecked=[]

        TocheckStruct=structArray()
        StructChecked=structArray()
        TocheckStruct.Procedure=procedure
        if TocheckStruct.Procedure==0:
            temp_targ_fori=np.array(np.loadtxt(str(rospy.get_param("/PosePath/drill"))+"/profi_tcp_pose1.txt", usecols=range(8)))
            Targ_fori = []
            numbers = json.loads(self.cluster_to_scan.value)
            for i in range(len(temp_targ_fori)):
              print(numbers)
              for n in numbers:
                if temp_targ_fori[i][0]==n:
                  Targ_fori.append(temp_targ_fori[i])

            Targ_fori = np.asarray(Targ_fori)
        elif TocheckStruct.Procedure==1:
            Targ_fori=np.array(np.loadtxt(str(rospy.get_param("/PosePath/drill"))+"/drill_tcp_pose2.txt", usecols=range(8)))
        elif TocheckStruct.Procedure==-5:
            Targ_fori=np.array(np.loadtxt(str(rospy.get_param("/PosePath/drill"))+"/Path_Calib_profi.txt", usecols=range(8)))

        if TocheckStruct.Procedure==0 or TocheckStruct.Procedure==1 or TocheckStruct.Procedure==-5:  
          Ordered_tag_fori=np.ones((len(Targ_fori),9))*(-2)
          Ordered_tag_fori[:,0:8]=Targ_fori
          print(Ordered_tag_fori)
  
  
  
          #Create the right structure
          for i in range(len(Ordered_tag_fori)):
              Tocheck=[]
              Tocheck=baseStruct()
              Tocheck.ID=i
              Tocheck.drillOp=drillbit
              Tocheck.label=int(Ordered_tag_fori[i,0])
              Tocheck.Pose.position.x=Ordered_tag_fori[i,1]
              Tocheck.Pose.position.y=Ordered_tag_fori[i,2]
              Tocheck.Pose.position.z=Ordered_tag_fori[i,3]
              Tocheck.Pose.orientation.w=Ordered_tag_fori[i,4]
              Tocheck.Pose.orientation.x=Ordered_tag_fori[i,5]
              Tocheck.Pose.orientation.y=Ordered_tag_fori[i,6]
              Tocheck.Pose.orientation.z=Ordered_tag_fori[i,7]
              Tocheck.reach=int(-2)
              TocheckStruct.structList.append(Tocheck)



        rospy.wait_for_service('reach_check')
        rospy.wait_for_service('path_plan')
        rospy.wait_for_service('state_request')
        print("services active")
        try:
            reach_check=rospy.ServiceProxy('reach_check', ReachCheck)
            path_plan=rospy.ServiceProxy('path_plan', PathPlan)
            ask_state=rospy.ServiceProxy('state_request', StateReq)
            if TocheckStruct.Procedure==0 or TocheckStruct.Procedure==1 or TocheckStruct.Procedure==-5:
                StructChecked=reach_check(TocheckStruct)
                print(StructChecked.ReachOut)
                print("#######################################################")
                GenPath=path_plan(StructChecked.ReachOut)
                print(GenPath.PlanOut)
                
                # # ## added for 3d cam dense
                # GenPath = PathPlanResponse()
                # GenPath.PlanOut = StructChecked.ReachOut
                # ###

                GenPath.PlanOut.FilePath= "/home/oguz/Desktop/TEMP/DENSE"
                finalreq=ask_state(GenPath.PlanOut)
                
            else:
                TocheckStruct.FilePath = "/home/oguz/Desktop/TEMP/SPARSE"
                finalreq=ask_state(TocheckStruct)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)    

    def buttonSetCurrentTool(self):
      print("Setting Current Tool")
      rospy.set_param("/DrillBit/actual", int(self.current_tool_value.value))
      print("Current Tool Mounted is : ", int(rospy.get_param("/DrillBit/actual")))
      return
    def buttonSparseAlignment(self):
          print('STARTING SPARSE ALIGNMENT')
          try:
            rospy.wait_for_service("sparse_alignment",timeout=1)
            sparse_serv= rospy.ServiceProxy("sparse_alignment", SparseAlignment)
            req =  SparseAlignmentRequest()
            req.mode = int(self.sparse_mode_value.value)
            req.Path= "/home/oguz/Desktop/TEMP/SPARSE"
            resp = sparse_serv(req)
            print(resp.error)
            return
          except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            return
    def buttonDenseAlignment(self):
          print('STARTING DENSE ALIGNMENT')
          try:
            rospy.wait_for_service("dense_alignment",timeout=1)
            dense_serv= rospy.ServiceProxy("dense_alignment", DenseAlignment)
            req = DenseAlignmentRequest()
            req.cluster_id = int(self.dense_cluster_value.value)
            req.Path= "/home/oguz/Desktop/TEMP/DENSE"
            resp = dense_serv(req)
            print(resp)
            return
          except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            return
    def buttonAxisClustering(self):
      print("STARTING AXIS EXTRACTION AND CLUSTERING")
      try:
          rospy.wait_for_service("axis_clustering",timeout=1)
          ax_serv= rospy.ServiceProxy("axis_clustering", ExtractionAndClustering)
          colors = json.loads(self.axis_colors.value)
          colors_req = []
          for cs in colors:
            c = color()
            c.r = float(cs[0])
            c.g = float(cs[1])
            c.b = float(cs[2])
            colors_req.append(c)

          req = ExtractionAndClusteringRequest()
          req.colors = colors_req
          resp = ax_serv(req)
          print(resp.error)
          return
      except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        return


    def buttonDenseScan3DCamera(self):
      #loading all the path
      filepath = "/home/oguz/Desktop/TEMP/X36"
      Targ_fori=np.array(np.loadtxt(str(rospy.get_param("/PosePath/drill"))+"/profi_tcp_pose1.txt", usecols=range(8)))
      num_old=0
      
      for i in range(len(Targ_fori)):
        num_new = Targ_fori[i,0]
        if not os.path.isdir(filepath+"/"+str(int(num_new))):
          os.mkdir(filepath+"/"+str(int(num_new)))
        filepath1 = filepath+"/"+str(int(num_new))
        rospy.set_param("FilePath/sparse", filepath1)
                #go to position
        s_gotopos = rospy.ServiceProxy("GoToPose", GoToPose)
        req = GoToPoseRequest()
        target_point = Targ_fori[i,1:8]
        target_point = [float(x) for x in target_point]
        print(target_point)
        req.pose = list(target_point)
        response = s_gotopos(req)
                #grab 3d photo
          
        rospy.wait_for_service('grab_3dpointcloudx36',timeout=1)
        print('grab 3d snapshot from 3d camera')
        
        grab3d = rospy.ServiceProxy("grab_3dpointcloud",Grab3DPointcloudx36)
        resp = grab3d()

        #self._pointcloud_callback(resp, filepath1,self.count)
        #save coords
        # getpos =  rospy.ServiceProxy("GetPose",GetPose)
        # resp = getpos()
        # pose = list(resp.pose[2:])
        # print(pose)
        # fCoord = open(filepath + "/Coords"+str(num_new)+".txt", "w")
        # fCoord.write("Number\tX [mm]\tY [mm]\tZ [mm]\tqw\tqz\tqy\tqz\n"+
        # str(num_new)+"\t"+str(pose[0])+"\t"+str(pose[1])+"\t"+str(pose[2])+"\t"+str(pose[3])+"\t"+str(pose[4])+"\t"+str(pose[5])+"\t"+str(pose[6]))
        # fCoord.close()

        self.count+=1
        if num_new!=num_old:
          self.count=0

        num_old = num_new
        
    # def SendZeroPLC(self):
    #     if self.PLC_connected == False:
    #           print('PLC is  disconnected')
    #           return
    #     else:
    #       rospy.wait_for_service('plc_command',timeout=1)
    #       print('UNLOADING')
    #       try:
    #         conn_plc= rospy.ServiceProxy("plc_command",CommandPLC)
    #         comando =  CommandPLCRequest()
    #         comando.command ="GVL_ROS.IRC5_ParteProgrammaAct"
    #         comando.value = 0
    #         resp = conn_plc(comando)
    #         if resp.res==1:
    #           self.PLC_connected =False
    #         print(resp.error)
    #         return
    #       except rospy.ServiceException as e:
    #         print("Service call failed: %s"%e)
    #         return


if __name__ == "__main__":   
    try:
      pyforms.start_app( FrankCommander )
    except rospy.ROSInterruptException:
      pass
