#! /usr/bin/python3
import FrankCamera 
import FrankCommons
import FrankUtilities#######
# from matplotlib import pyplot as plt
import rospy#
# from pyntcloud import PyntCloud
import pandas as pd
import open3d as o3d

import numpy as np
# import sys
# sys.path.append("/usr/local/lib/python3.7/dist-packages/")
import events

import cx.cx_base as cxb
import cx.cx_cam as cxc
import cx.cx_3d as c3d
import time
import math
#import rospy
import pickle
import os
import cv2
from pathlib import Path
from frank.srv import *
calib_fname = "calib_download.dat"
class newProfilometer():
    
    CalibrationFile = str(Path(__file__).resolve().parent)+'/C5-4090CS18-490.deb'
    Model = "C5-4090CS18-490"
    IpToSearch = "10.0.0.239" #An IP of a network to be searched for devices
    MaskToSearch = "255.255.255.0"

    device = None #Device currently open
    calib = None
    Connected = False
    AcquisitionStarted = False
    TriggerStarted = False
    RequestedStop = False
    Append = False
    pose_robot = []
    counter_pose = 0
    RAPIDTriggerFreq = 100 #Hz, derives from RAPID cycle time
    ProfiReady = False
    timestamp_start = 0
    timestamp_stop = 0
    id_acqu=0
    Filepath = "/home/c301/Desktop/TEMP/"
    #Session parameters
    SC = 0
    uri = "" #uri of the open device

    fFOVx = 900 # maximum x-FOV (mm)

    IR_EZ = math.radians(0)#(90)
    IR_EY = math.radians(0)
    IR_EX = math.radians(180)
    IdealProfilometerCoordSystem = FrankUtilities.Euler2RotMatrix([IR_EZ, IR_EY, IR_EX])
    R_EZ = math.radians(0.67525)#(89.17594764921905)#(90.0077759)#(89.513)#(89.33704852054858)#(89.17594764921905)
    R_EY = math.radians(-0.10451)#(-0.101484388)#(-0.010914760143877267)#(-0.11237917549682697)
    R_EX = math.radians(179.8448)#(181.1319056)#(179.993)#(179.9128511490148)#(180.03408582397668)
    ProfilometerCoordSystem = FrankUtilities.Euler2RotMatrix([R_EZ, R_EY, R_EX])
    ProfilometerOffset =[]#[48.924, -15, -72.91437]#[48.924, -14.8584, -72.91437]# 205.06482242638083]#50.56595726497499]# [1.919597951,49.27719208,52.3350607]#[1.502,48.916,50.774]#[1.0717818961710721,48.072654304144066,51.03132584386907] #[1.0714803654320781, 48.64106065763202, 50.56595726497499]  #Offset of the profilmeter with respect to tool origin #Check Sign

    ProfilometerZero = [0.0, 0.0, 744.0+95.1] #744 i the working distance (pag. 15 of the manual) and 95.1 is the height of the profilometer (pag. 22)

    MaximumWidth = 450 #Maximum Width of the blade, used to defined multiple passes to scan a workpiece

    myevents = events.Events(('PointCloudProgress'))

    def __init__(self, SensorModel = "C5-4090CS18-490"):
        self.Model = SensorModel
        pass

    def Connect(self):
        '''
        Connects to the first device of specified Model
        '''
        if self.Connected == True: return 0
        print("Searching Profilometers")
        search_status = cxc.cx_dd_findDevices("", 5000, cxc.CX_DD_USE_GEV | cxc.CX_DD_USE_GEV_BROADCAST)
        print("Connecting to profilometer")
        if search_status == cxb.CX_STATUS_OK:
            n_dev = cxc.cx_dd_getNumFoundDevices()[1]
            print(n_dev)
            if n_dev > 0:
                self.uri = ""
                for n in range(n_dev):
                    if cxc.cx_dd_getParam(n, "Model")[1] == self.Model:
                        self.uri = cxc.cx_dd_getParam(n, "uri")[1]
                        break
                if self.uri == "":
                    print("Profilometer not found")
                    self.SC = -8
                dev_status, self.device = cxc.cx_openDevice(self.uri)
                print("Connection status: " + cxb.cx_status_getText(dev_status))
                self.SC = dev_status
                self.Connected = True
                #self.calib = c3d.cx_3d_calib_load(self.CalibrationFile, "factory", c3d.CX_3D_CALIB_FORMAT_AUTO)[1]
                #c3d.cx_3d_calib_set(self.calib, c3d.CX_3D_PARAM_METRIC_IDV, np.nan)
                ### get calibration fromdevice
                FileSelector = "UserData"             # UserData,  CalibrationFactory, CalibrationUser
                result = cxc.cx_downloadFile(self.device,FileSelector,calib_fname)
                if result != cxb.CX_STATUS_OK:
                    print("cx_downloadFile returned error %d" % result)
                    exit(0)
                result, self.calib = c3d.cx_3d_calib_load(calib_fname, "factory", c3d.CX_3D_CALIB_FORMAT_AUTO)
                if result != cxb.CX_STATUS_OK:
                    print("cx_3d_calib_load returned error %d" % result)
                    exit(0)
                c3d.cx_3d_calib_set(self.calib, c3d.CX_3D_PARAM_METRIC_IDV, np.nan)
            else:
                print("Profilometer not found")
                self.SC = -8
        else:
            print("Error in device search: " + cxb.cx_status_getText(search_status))
            self.SC = -8

        return self.SC

    def Disconnect(self):
        '''
        Disconnect from Device
        '''
        if not self.Connected:
            print ("Not Connected to device")
            return -1

        if self.AcquisitionStarted:
            self.RequestedStop = True
            time.sleep(1)
            cxc.cx_stopAcquisition(self.device)
            self.AcquisitionStarted = False
        cxc.cx_closeDevice(self.device)
        c3d.cx_3d_calib_release(self.calib)
        self.device = None
        self.uri = None
        self.Connected = False
        self.SC = 0
        return 0

    def InitializeProfilometer(self, ProfilesPerFrame : int, xScale : float):
        '''
        ProfilesPerFrame - number of profiles to be acquired for each "frame"
        xScale - resolution (in mm) along blade direction
        '''
        if not self.Connected:
            print ("Not Connected to device")
            return -1

        # Parameters (Pay attention: all functions return a parameter)
        result = cxc.cx_setParam(self.device, "PixelFormat", "Mono16")
        result = cxc.cx_setParam(self.device, "Width", 4096)
        result = cxc.cx_setParam(self.device, "OffsetX", 0)
        result = cxc.cx_setParam(self.device, "ReverseX", 0)  
        result = cxc.cx_setParam(self.device, "ReverseY", 0)

        result = cxc.cx_setParam(self.device, "AcquisitionMode", "Continuous")
        result = cxc.cx_setParam(self.device, "ExposureTime", 2000)

        result = cxc.cx_setParam(self.device, "AoiHeight", 2048) # 3072
        result = cxc.cx_setParam(self.device, "AoiOffsetY", 1024) # 0
        result = cxc.cx_setParam(self.device, "AoiThreshold", 520)
        result = cxc.cx_setParam(self.device, "CameraMode", "FIRPeak")
        result = cxc.cx_setParam(self.device, "ProfilesPerFrame", ProfilesPerFrame)
        #result = cxc.cx_setParam(self.device, "SubpixelBits", 4)
        
        result = cxc.cx_setParam(self.device, "AcquisitionStatusSelector", "AcquisitionTriggerWait")
        result = cxc.cx_setParam(self.device, "FramePeriod", 8000) #10000
        result = cxc.cx_setParam(self.device, "LightBrightness", 50.0)
        result = cxc.cx_getParam(self.device, "AcquisitionLineRate")
        result = cxc.cx_setParam(self.device, "AcquisitionLineRate", self.RAPIDTriggerFreq) #ReadOnly

        result = cxc.cx_setParam(self.device, "SequencerMode", "FreeRun")
        result = cxc.cx_setParam(self.device, "ProfileTriggerMode", "CameraInput1")

        offset = np.array([-self.fFOVx/2, 0.0, 0.0], dtype = np.float64)
        scale = np.array([xScale, 1, 1], dtype = np.float64)
        result = c3d.cx_3d_calib_set(self.calib, c3d.CX_3D_PARAM_METRIC_O, offset)
        result = c3d.cx_3d_calib_set(self.calib, c3d.CX_3D_PARAM_METRIC_S, scale)

        return 0

    def InitializeProfilometer_stepbystep(self, ProfilesPerFrame : int, xScale : float):
        '''
        ProfilesPerFrame - number of profiles to be acquired for each "frame"
        xScale - resolution (in mm) along blade direction
        '''
        if not self.Connected:
            print ("Not Connected to device")
            return -1

        # Parameters (Pay attention: all functions return a parameter)
        result = cxc.cx_setParam(self.device, "PixelFormat", "Mono16")
        result = cxc.cx_setParam(self.device, "Width", 4096)
        result = cxc.cx_setParam(self.device, "OffsetX", 0)
        result = cxc.cx_setParam(self.device, "ReverseX", 0)  
        result = cxc.cx_setParam(self.device, "ReverseY", 0)

       # result = cxc.cx_setParam(self.device, "AcquisitionMode", "Continuous")
        result = cxc.cx_setParam(self.device, "AcquisitionMode", "SingleFrame")
        result = cxc.cx_setParam(self.device, "ExposureTime", 2000)

        result = cxc.cx_setParam(self.device, "AoiHeight", 2048) # 3072
        result = cxc.cx_setParam(self.device, "AoiOffsetY", 1024) # 0
        result = cxc.cx_setParam(self.device, "AoiThreshold", 520)
        result = cxc.cx_setParam(self.device, "CameraMode", "FIRPeak")
        result = cxc.cx_setParam(self.device, "ProfilesPerFrame", 1)
        #result = cxc.cx_setParam(self.device, "SubpixelBits", 4)
        
        result = cxc.cx_setParam(self.device, "AcquisitionStatusSelector", "AcquisitionActive")
       # result = cxc.cx_setParam(self.device, "FramePeriod", 8000) #10000
        result = cxc.cx_setParam(self.device, "LightBrightness", 50.0)
        
        result = cxc.cx_setParam(self.device, "SequencerMode", "FreeRun")
        result = cxc.cx_setParam(self.device, "ProfileTriggerMode","FreeRun" )

        offset = np.array([-self.fFOVx/2, 0.0, 0.0], dtype = np.float64)
        scale = np.array([xScale, 1, 1], dtype = np.float64)
        result = c3d.cx_3d_calib_set(self.calib, c3d.CX_3D_PARAM_METRIC_O, offset)
        result = c3d.cx_3d_calib_set(self.calib, c3d.CX_3D_PARAM_METRIC_S, scale)

        return 0
    
    def InitializeProfilometer_continous(self,xScale):
        '''
        xScale - resolution (in mm) along blade direction
        '''
        if not self.Connected:
            print ("Not Connected to device")
            return -1

        # Parameters (Pay attention: all functions return a parameter)
        result = cxc.cx_setParam(self.device, "PixelFormat", "Mono16")
        result = cxc.cx_setParam(self.device, "Width", 4096)
        result = cxc.cx_setParam(self.device, "OffsetX", 0)
        result = cxc.cx_setParam(self.device, "ReverseX", 0)  
        result = cxc.cx_setParam(self.device, "ReverseY", 0)

        result = cxc.cx_setParam(self.device, "AcquisitionMode","SingleFrame")
        result = cxc.cx_setParam(self.device, "ExposureTime", 2000)

        result = cxc.cx_setParam(self.device, "AoiHeight", 2048) # 3072
        result = cxc.cx_setParam(self.device, "AoiOffsetY", 1024) # 0
        result = cxc.cx_setParam(self.device, "AoiThreshold", 520)
        result = cxc.cx_setParam(self.device, "CameraMode", "FIRPeak")
        result = cxc.cx_setParam(self.device, "ProfilesPerFrame", 1)
        #result = cxc.cx_setParam(self.device, "SubpixelBits", 4)
        
        result = cxc.cx_setParam(self.device, "AcquisitionStatusSelector", "AcquisitionActive")
       # result = cxc.cx_setParam(self.device, "FramePeriod", 8000) #10000
        result = cxc.cx_setParam(self.device, "LightBrightness", 50.0)
        
        result = cxc.cx_setParam(self.device, "SequencerMode", "FreeRun")
        result = cxc.cx_setParam(self.device, "ProfileTriggerMode","FreeRun" )

        offset = np.array([-self.fFOVx/2, 0.0, 0.0], dtype = np.float64)
        scale = np.array([xScale, 1, 1], dtype = np.float64)
        result = c3d.cx_3d_calib_set(self.calib, c3d.CX_3D_PARAM_METRIC_O, offset)
        result = c3d.cx_3d_calib_set(self.calib, c3d.CX_3D_PARAM_METRIC_S, scale)

        return 0
    def Acquire_trigger_test(self):
        i = 0
        self.AcquisitionStarted = True
        self.RequestedStop = False
        while True:
            print(i)
            i+=1
            if self.RequestedStop:
                break
        self.AcquisitionStarted = False
        return 1

    def Acquire_trigger(self,trig_starter, SavePath='', SaveToFile=False, RobotLinSpeed = 100, xScale = 0.125, ProfilesPerFrame =100):    ###### xScale=0.2 ######## profilesperframe = 100
        '''
        robot: reference to class robot to start the trigger
        sampletime: sample time currently setted
        xScale: x-resolution (mm), is just the resampling base
        SaveToFile: boolean that specified if file saving is enabled
        '''

        if not self.Connected:
            print ("Not Connected to device")
            return -1

        # Parameters (Pay attention: all functions return a parameter)
        self.InitializeProfilometer(ProfilesPerFrame, xScale)
        result  = cxc.cx_freeBuffers(self.device)
        # Acquisition
        result = cxc.cx_allocAndQueueBuffers(self.device, 10)
        result = cxc.cx_startAcquisition(self.device)
        self.AcquisitionStarted = True
        
        Profile = list()
        RangeImg = list()
        NN = 0
        self.RequestedStop = False
        while True:
            if NN == 0:
                print("Start Profilometer requested")
                #robot.start_stop_profile(1, round(1 / self.RAPIDTriggerFreq, 3)) #Maximum sampletime is 0.01
                #trigger_pub.Publish(1)
                self.ProfiReady = True
                self.Append = True # START SAVING POSE BEFORE TRIGGER STARTED
                req = CommandPLCRequest()
                req.value = 50
                req.command = "GVL_ROS.PL_ProfilTrig_D"
                stamp1 = rospy.get_rostime()
                resp = trig_starter(req)
                self.timestamp_start = resp.stamp
                print("waiting for trigger to start")  # start the trigger when# arrived here the profilometer i ready
                while self.TriggerStarted == False:
                    a=0
                    #send message to old_frank to start the trigger
                #self.Append = True    
                print("trigger started, starting acquisition")    

            tt = time.time()
            buffer = cxc.cx_waitForBuffer(self.device, 10000)[1] #Wait for image in buffer
            ttt = time.time()
            print("Buffer time: " + str(ttt-tt))

            cxRangeImg = cxc.cx_getBufferImage(buffer, 0)[1] #Extract image from buffer
            zMap = cxb.cx_img_t(ProfilesPerFrame + 1, int(self.fFOVx/xScale), cxb.CX_PF_COORD3D_C32f) #Convert cxRangeImg in format CX_PF_COORD3D_C32f #ProfilesPerFrame + 1 because the last one is always empty
            result = c3d.cx_3d_range2rectifiedC(self.calib, cxRangeImg, zMap, c3d.CX_3D_METRIC_MARK_Z_INVALID_DATA 
                                                | c3d.CX_3D_METRIC_INTERP_IDW)
            cxc.cx_queueBuffer(buffer)

            #zMap.data contiene "ProfilesPerFrame" profili, vanno salvati tutti o comunque controllato il size di data
            for i in range(0, len(zMap.data) - 1): # -1 because the last one is always empty
                Profile.append(zMap.data[i])
                #RangeImg.append(cxRangeImg.data[i])
                if np.sum(Profile[len(Profile) - 1]) == 0:
                    print("Profile = 0, N = " + str(NN) + ", i = " + str(i))

            #print("Cycling... (" + str(NN) + ")")
            cxc.cx_queueBuffer(buffer)

            if self.RequestedStop:
                break

            NN = NN + 1
            pass

        print("Stop Profilometer requested")
 
        ## send PLC command to stop trigger
        req = CommandPLCRequest()
        req.value = 0
        req.command = "GVL_ROS.PL_ProfilTrig_D"
        stamp2 = rospy.get_rostime()
        resp = trig_starter(req) #
        self.timestamp_stop = resp.stamp
        print("waiting to stop the trigger")
        while self.TriggerStarted == True:
            b=0
        self.Append == False
        print("Stop Profilometer done")
        time.sleep(0.2)
        cxc.cx_stopAcquisition(self.device)
        result  = cxc.cx_freeBuffers(self.device)
        self.AcquisitionStarted = False
        numX = len(Profile[0]) * xScale
        #XX = np.arange(numX / 2, -numX / 2, -xScale)
        XX = np.arange(-numX / 2, numX / 2, xScale)
        YY = np.zeros(len(Profile)) #np.arange(0, len(Profile) * Yres, Yres) #MUST BE ZERO

        #cv2.imwrite(SavePath.replace(".txt", ".tiff"), RangeImg)

        if SaveToFile:
            #SAVE PROFILOMETER DATA
            print("Profilometer acquisition: saving to file")
            file = open(SavePath+str(self.id_acqu)+'.txt',"w")

            stringList = '\t'.join([str(item) for item in XX]) + "\n"
            file.write(stringList)
            #Yres = RobotLinSpeed / self.RAPIDTriggerFreq
            #YY = np.arange(0, len(Profile) * Yres, Yres) #Nominal values
            stringList = '\t'.join([str(item) for item in YY]) + "\n"
            file.write(stringList)

            for i in range(0, len(Profile)):
                stringList = '\t'.join([str(item) for item in Profile[i]]) + "\n"
                file.write(stringList)
        
            file.close()
            print("Profilometer acquisition: saving completed")
            #SAVING ROBOT POSE ACQUIRED
            print("print Saving Robot Poses: ")
            file = open(SavePath+"_rob"+str(self.id_acqu)+".txt","w")
            np.savetxt(file,self.pose_robot)
            file.close()
            print("Profilometer pose acquisition: saving completed")
            print("print Saving trigger time stamp")
            file = open(SavePath+"_stamp"+str(self.id_acqu)+".txt","w")
            file.write(str(stamp1.to_sec())+" "+ str(stamp1.to_nsec())+ 
                    "\n"+str(self.timestamp_start.to_sec())+" "+ str(self.timestamp_start.to_nsec())+ 
                    "\n"+ str(stamp2.to_sec())+" "+ str(stamp2.to_nsec())+ 
                     "\n"+ str(self.timestamp_stop.to_sec())+" "+ str(self.timestamp_stop.to_nsec()))
            file.close()
            print("Profilometer timestamp acquisition: saving completed")
        ZZ = np.array(Profile)
        self.pose_robot = []
        self.counter_pose = 0
        self.id_acqu+=1
        return [XX, YY, ZZ]
    
    def AcquireContinous(self, profile_pub, xScale):
 
        if not self.Connected:
            print ("Not Connected to device")
            return -1

        # Parameters (Pay attention: all functions return a parameter)
        self.InitializeProfilometer_continous(xScale)

        # Acquisition
        result = cxc.cx_allocAndQueueBuffers(self.device, 100)
        result = cxc.cx_startAcquisition(self.device)
        self.AcquisitionStarted = True
        
        Profile = list()
        RangeImg = list()
        NN = 0
        self.RequestedStop = False
        rate = rospy.Rate(1)
        while True:
            print("Start Profilometer requested")
            buffer = cxc.cx_waitForBuffer(self.device, 10000)[1]
            cxRangeImg = cxc.cx_getBufferImage(buffer, 0)[1]
            zMap = cxb.cx_img_t(1, int(self.fFOVx/xScale), cxb.CX_PF_COORD3D_C32f)
            print(zMap.data[0])
            if self.RequestedStop == True:
                break
            rate.sleep()
        pass
 
    def Acquire_profilo(self, pub_profilo, xScale = 1, ProfilesPerFrame = 1):    ###### xScale=0.2 ########
        
        if not self.Connected:
            print ("Not Connected to device")
            return -1

        # Parameters (Pay attention: all functions return a parameter)
        self.InitializeProfilometer_stepbystep(ProfilesPerFrame, xScale)

        # Acquisition
        result = cxc.cx_allocAndQueueBuffers(self.device, 10)
        # if result!=cxb.CX_STATUS_OK:
        #     print("cx_allocAndQueueBuffers(%d) returned error %d" % (hDev, result))
        # result = cxc.cx_startAcquisition(self.device)
        # if result!=cxb.CX_STATUS_OK:
        #     print("cx_startAcquisition(%d) returned error %d" % (hDev, result))
        self.AcquisitionStarted = True
        self.RequestedStop = False
        rate  = rospy.Rate(1)
        while True:    
            result, buffer = cxc.cx_waitForBuffer(self.device, 10000) #Wait for image in buffer
            if result != cxb.CX_STATUS_OK:
                print("cx_waitForBuffer returned error %d" % (result))
            result, Img = cxc.cx_getBufferImage(buffer, 0) #Extract image from buffer
            if result != cxb.CX_STATUS_OK:
                print("cx_getBufferImage returned error %d" % (result))
        
            zMap = cxb.cx_img_t(ProfilesPerFrame + 1, int(self.fFOVx/xScale), cxb.CX_PF_COORD3D_C32f) #Convert cxRangeImg in format CX_PF_COORD3D_C32f #ProfilesPerFrame + 1 because the last one is always empty
            result = c3d.cx_3d_range2rectifiedC(self.calib, Img, zMap, c3d.CX_3D_METRIC_MARK_Z_INVALID_DATA 
                                                    | c3d.CX_3D_METRIC_INTERP_IDW)

            profilo = zMap.data[0]
            print(profilo)
            #bol = np.isnan(profilo)
            #profilo = profilo[~bol]
            #plt.figure()
            #plt.plot(profilo)
            #plt.savefig('E:/frame.png')
            #print('ho scattato un frame')
            result  = cxc.cx_queueBuffer(buffer)
            result = cxc.cx_stopAcquisition(self.device)
            #cleanup
            result = cxc.cx_freeBuffers(self.device)

            if self.RequestedStop == True:
                result = cxc.cx_stopAcquisition(self.device)
                break
            rate.sleep()
            #print('mi fermo? : ', self.RequestedStop)
        #pub_profilo.publish(profilo)       
        return #  profilo, xScale

    def ComputeMultiPath(self, P1, P2, P3, P4, distance):
        '''
        Compute the path considering the maximum width of the laser blade.
        The rotation matrix is not changing during the path as start end end points describe segments parallel to the longest side of the rectangle
        '''

        res = FrankCamera.GetRectFrom4Vertexes(P1,P2,P3,P4)
        if res == -1:
            return
        rect = res[0]
        offset = res[1]
        RT = res[2]

        [v1, v2, v3] = FrankCommons.RotMatrixToVersors(RT) #v3 is normal versor
        #target = FrankUtilities.Target3rdVersor2PosRT(offset, v3, v2, distance)
        CO = self.ProfilometerOffset

        target = RT

        #Start and Stop points must be calculated
        FP1 = [0, 0, 0]
        FP2 = [0, 0, 0]
        FP3 = [0, 0, 0]
        FP4 = [0, 0, 0]
        N = 0
        if rect[0] >= rect[1]:
            N = math.ceil(rect[1] / self.MaximumWidth)
            #P1 with P4 as start and P2 with P3 as end
            #RT is ok
            FP1 = P1
            FP2 = P3
            FP3 = P2
            FP4 = P4
            rot = FrankUtilities.Euler2RotMatrix([math.radians(90), 0, 0])
            RT = FrankCommons.MatrixProduct(rot, self.IdealProfilometerCoordSystem)
            RT = FrankCommons.MatrixProduct(target, RT)
            pass
        
        else:
            N = math.ceil(rect[0] / self.MaximumWidth)
            #P1 with P2 as start and P3 with P4 as end
            #RT must be rotated 90 degs
            FP1 = P1
            FP2 = P3
            FP3 = P4
            FP4 = P2
            RT = FrankCommons.MatrixProduct(target, self.IdealProfilometerCoordSystem)
            pass

        offset = FrankCommons.MatrixArrayProduct(RT, CO)
        start = []
        end = []
        resRT = []
        for i in range(N):
            num = (i+1)/(N+1)
            stemp = [FP1[0]+(FP4[0]-FP1[0])*num - offset[0], FP1[1]+(FP4[1]-FP1[1])*num - offset[1], FP1[2]+(FP4[2]-FP1[2])*num - offset[2]]
            etemp = [FP3[0]+(FP2[0]-FP3[0])*num - offset[0], FP3[1]+(FP2[1]-FP3[1])*num - offset[1], FP3[2]+(FP2[2]-FP3[2])*num - offset[2]]

            stemp = FrankUtilities.Target3rdVersor2PosRT(stemp, v3, v2, distance) #RT already calculated, used to compute correct distance from rect
            etemp = FrankUtilities.Target3rdVersor2PosRT(etemp, v3, v2, distance) #RT already calculated, used to compute correct distance from rect

            stemp = stemp[0]
            etemp = etemp[0]

            dx = etemp[0] - stemp[0]
            dy = etemp[1] - stemp[1]
            dz = etemp[2] - stemp[2]
            length = math.sqrt(dx**2 + dy**2 + dz**2)

            newend = [stemp[0] + dx * 1.1, stemp[1] + dy * 1.1, stemp[2] + dz * 1.1]
            stemp = [etemp[0] - dx * 1.1, etemp[1] - dy * 1.1, etemp[2] - dz * 1.1]
            etemp = newend

            stemp[0] = round(stemp[0], 3) #micron
            stemp[1] = round(stemp[1], 3) #micron
            stemp[2] = round(stemp[2], 3) #micron
            etemp[0] = round(etemp[0], 3) #micron
            etemp[1] = round(etemp[1], 3) #micron
            etemp[2] = round(etemp[2], 3) #micron

            start.append(stemp)
            end.append(etemp)
            resRT.append(RT)
            pass

        return [start, end, resRT]

    def ReadRobotCoordinateInputFile(self, filePath : str):
        '''
        Returns an array containig:
        Num - progressive number of acquired coordinate/profilometer trigger
        Coord - [X, Y, Z] coordinate of robot tool
        Quat - [q0, qx, qy, qz] coordinate of robot tool
        '''
        #File loading of robot coordinate ******************************************************************************
        file = open(filePath,"r")
        line = file.readline() #reads the header

        Num = []
        Coord = []
        Quat = []

        line = file.readline()
        id = 0
        while line != "":
            divisi = line.split("\t")
            Num.append(int(divisi[0]))
            rX = (float(divisi[1]))
            rY = (float(divisi[2]))
            rZ = (float(divisi[3]))
            rqw = (float(divisi[4]))
            rqx = (float(divisi[5]))
            rqy = (float(divisi[6]))
            rqz = (float(divisi[7]))
            Coord.append([rX, rY, rZ])
            Quat.append([rqw, rqx, rqy, rqz])
            line = file.readline()
            id = id + 1
        file.close
        #File loading of robot coordinate ******************************************************************************

        return [Num, Coord, Quat]

    def ReadPointCloudInputTextFile(self, filePath : str):
        '''
        Point cloud: X as first row, Y as second row, Z as matrix.
        Returns a standard point cloud matrix ([[x1, y1, z1], [x2, y2, z2], ...]
        '''

        file = open(filePath,"r")
        line = file.readline()
        XX = []
        YY = []
        ZZ = []
        id = 0
        while line != "":
            divisi = line.split("\t")
            if XX == []:
                XX = [float(ele) for ele in divisi]
            elif YY == []:
                YY = [float(ele) for ele in divisi]
            else:
                ZZ.append([float(ele) for ele in divisi])
                pass

            line = file.readline()
            id = id + 1
        file.close()
        #File loading of "old point cloud" *****************************************************************************

        return [XX, YY, ZZ]

    def Build_RobotMultiPointCloud(self,rC, XX, YY, ZZ, SavePath):
        '''
        Input elements are Lists
        '''

        points = []

        for i in range(len(rC)):
            NCoord = len(rC[i][0])
            NY = len(YY[i])
            N = min(NCoord, NY)

            file = ""
            if len(rC) > 1:
                file = SavePath.replace(".ply", "_" + str(i) + ".ply")
    
            temp = self.BuildPointCloud_RobotCoordinate(rC[i][0][0:N], rC[i][1][0:N], rC[i][2][0:N], XX[i], YY[i][0:N], ZZ[i][0:N], file)
            if i == 0:
                points = temp
            else:
                points = np.append(points, temp, axis = 0)
            pass

        self.Write_PLY(points, SavePath)

        return

    def BuildPointCloud_RobotCoordinate(self, Num, Coord, Quat, XX, YY, ZZ, SavePath):

        NCoord = len(Num) # -> Should be equal to number of acquired profiles NY
        NX = len(XX) #len(ZZ[0])
        NY = len(YY) #len(ZZ) -> number of acquired profiles
        N = NX * NY
        points = np.zeros((N,3))
        id = 0
        id2 = 0 #debug
        percprec = 0 #debug
        self.myevents.PointCloudProgress(0) #debug
        zero = self.ProfilometerZero
        
        #PROPOSED NEW ALGOTITHM (working)
        profiRT = np.array(self.ProfilometerCoordSystem)
        #print(profiRT)#############################################
        profiRT = np.transpose(profiRT)
        offset = self.ProfilometerOffset

        for i in range(0, min(NCoord, NY)): #(0, NY): #THIS MUST BE SOLVED WITH PROPER SYNC BETWEEN PROFILOMETER AND COORD
            id2 = id2 + 1 #debug
            perc = int(id2/min(NCoord, NY)*100) #debug
            if perc!=percprec: #debug
                percprec=perc #debug
                self.myevents.PointCloudProgress(perc) #debug

            points[i*NX:(i+1)*NX, 0] = np.array(XX)
            points[i*NX:(i+1)*NX, 1] = np.array(YY[i]) * np.ones((NX))
            points[i*NX:(i+1)*NX, 2] = np.array(ZZ[i])
            points[i*NX:(i+1)*NX, :] = points[i*NX:(i+1)*NX, :].dot(profiRT) + zero + offset
            RT = FrankUtilities.Quaternion2RotMatrix(list(Quat[i])) #Rotation matrix of the Tool
            RT = np.transpose(np.array(RT))
            cTemp = (np.array(Coord[i])).reshape((1,3))
            points[i*NX:(i+1)*NX, :] = points[i*NX:(i+1)*NX, :].dot(RT) + cTemp
            pass

        bol = ~np.isnan(points[:,2])
        points = points[bol,:]

        if SavePath != "":
            self.Write_PLY(points, SavePath)
            
        return points

    def Write_PLY(self, points, filePath):
        '''
        create a Point Cloud (with PyntCloud) and writes it to file
        '''
        SaveMesh = False
        ShowPointCloud = False
        UsePyntCloud = False

        #PLY file saving ***********************************************************************************************
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)

        if UsePyntCloud:
            frame = pd.DataFrame(points, columns=["x", "y", "z"])
            cloud = PyntCloud(frame)
            cloud.to_file(filePath)
        else:        
            o3d.io.write_point_cloud(filePath, pcd, write_ascii=False, compressed=True, print_progress=False)
        #PLY file saving ***********************************************************************************************

        if ShowPointCloud:
            downpcd = pcd.voxel_down_sample(voxel_size=0.05)
            o3d.visualization.draw_geometries([downpcd])

        if SaveMesh:
            pcd.estimate_normals()

            poisson_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=9, width=0, scale=1.1, linear_fit=False)[0]
            bbox = pcd.get_axis_aligned_bounding_box()
            p_mesh_crop = poisson_mesh.crop(bbox)
            
            o3d.io.write_triangle_mesh(filePath.replace(".ply", "_mesh.ply"), p_mesh_crop)
            o3d.visualization.draw_geometries([p_mesh_crop])

        return

