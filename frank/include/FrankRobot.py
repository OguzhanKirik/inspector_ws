import FrankCamera
import FrankLight
import FrankUtilities
import FrankStitcher
import FrankCommons
import FrankProfilometer

import abb
import threading
import socket
import time
import math

import struct

import cv2

#from pyntcloud import PyntCloud
#import pandas as pd
#import numpy as np
#import open3d as o3d

import events

class newFrank():

    myevents = events.Events(('CameraCalib_NewImage', 'WorkingArea_NewImage', 'NewRobotCoord', 'PointCloudProgress'))

    #Dichiarazione variabili ****************************************************
    robot = None                    #Variabile globale per accedere al Robot
    light = None                    #Variabile globale per accedere alla luce
    camera = FrankCamera.newCamera()                   #Variabile globale per accedere alla camera
    profilometer = FrankProfilometer.newProfilometer()
    stitcher = None                 #Variable to access Stitcher

    #robotIP = "127.0.0.1"       #IP del robot
    robotIP = "192.168.125.1"   #IP del robot
    toolLen = 200    #lunghezza del tool (Z tool coordinate)

    LoggerPort = 5001
    LoggerSampleTime = 0.01
    CoordinateLog = [[], [], []]
    CoordinateLogStarted = False
    CoordinateLogFileName = "C:/Coords.txt"
    CoordinateLogSaveToFile = False

    Connesso = False #Non cambiare
    #exitFlag = False #Non cambiare

    ConnessoLuce = False #Non cambiare
    lightIP = "192.168.125.2"   #IP della luce
    lightIntensity = 12
    lightChannel = 1
    lightPort = 40001

    ConnessoCamera = False

    ConnessoProfilometer = False

    #Dichiarazione variabili ****************************************************

    # Da finire
    def __init__(self, IProbot = "192.168.125.1", IPlight = "192.168.125.2"):
        self.robotIP = IProbot
        self.lightIP = IPlight

    def LogCoord(self):
        
        fCoord = None

        while self.Connesso:
            data = self.logsock.recv(300)
            N = round(len(data) / 30)
            nums = []
            coords = []
            for i in range(0, N):
                id = i * 30
                intVal = struct.unpack('h',data[id:id+2])[0]
                id = id + 2
                floatVals = struct.unpack('fffffff',data[id:4*7+id])
                nums.append(intVal)
                coords.append(floatVals)
                pass

            for i in range(0, len(nums)):
                if nums[i] != -1:
                    if not self.CoordinateLogStarted:
                        self.CoordinateLog = [[], [], []]
                        self.CoordinateLogStarted = True
                    if self.CoordinateLogSaveToFile:
                        if fCoord == None:
                            fCoord = open(self.CoordinateLogFileName, "w") #TO BE REMOVED
                            fCoord.write("Number\tX [mm]\tY [mm]\tZ [mm]\tqw\tqx\tqy\tqz\n") #TO BE REMOVED

                        fCoord.write(str(nums[i]) + "\t" + str(coords[i][0]) + "\t" + str(coords[i][1]) + "\t"
                                      + str(coords[i][2]) + "\t" + str(coords[i][3]) + "\t" + str(coords[i][4]) + "\t"
                                       + str(coords[i][5]) + "\t" + str(coords[i][6]) + "\n") #TO BE REMOVED
                    self.CoordinateLog[0].append(nums[i])
                    self.CoordinateLog[1].append([coords[i][0], coords[i][1], coords[i][2]])
                    self.CoordinateLog[2].append([coords[i][3], coords[i][4], coords[i][5], coords[i][6]])
                else:
                    if not fCoord is None:
                        fCoord.close
                        fCoord = None
                        pass
                    self.CoordinateLogStarted = False


            self.myevents.NewRobotCoord(nums, coords)

        self.logsock.shutdown(socket.SHUT_RDWR)

    def Connect(self):
        '''
        Connect to robot and light.
        Returns 0 when robot and light are connected
        Returns -1 if robot is not connected
        Returns -2 if light is not connected
        Returns -4 if camera is not connected
        Returns -8 if profilomer is not connected
        '''
        if self.Connesso: return 0

        try:
            self.robot = abb.Robot(ip = self.robotIP)
            #self.robot.set_tool([[0, 0, self.toolLen], [1, 0, 0, 0]]) #La lunghezza del tool è fino al sensore della camera
            self.SetToolLength(self.toolLen)
            print("Robot connesso")
            self.Connesso = True
            self.robot.start_stop_profile(0, self.LoggerSampleTime) #New Motion server required
        
            pass
        except :
            print("Errore nella connessione al robot, indirizzo IP: " + self.robotIP)
            self.Connesso = False
            pass


        if self.Connesso:
            try:
                self.logsock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.logsock.settimeout(5)
                self.logsock.connect((self.robotIP, self.LoggerPort))
                t3 = threading.Thread(target = self.LogCoord)
                t3.start()
                print("connected to Logger")       
                pass
            except :
                print("Logger not connected")
                pass
            res = self.Connect_Light()
            res = res + self.Connect_Camera()
            res = res + self.Connect_Profilometer()
            return res
        else:
            return -1

    def Connect_Light(self):
        '''
        Returns 0 if light is connected.
        Returns -2 if an error occured
        '''
        print("Connecting to light")
        try:
            self.light = FrankLight.newLight(self.lightIP, self.lightPort)
            self.light.setIntensity(self.lightChannel, self.lightIntensity)
            self.ConnessoLuce = True
            print("light connected")

            pass
        except:
            print("Errore nella connessione alla luce, indirizzo IP: " + self.lightIP)
            self.ConnessoLuce = False

            pass

        if self.ConnessoLuce:
            return 0
        else:
            return -2

    def Connect_Camera(self):
        res = self.camera.Connect()
        if res < 0:
            return res
        self.ConnessoCamera = True
        return 0

    def Connect_Profilometer(self):
        res = self.profilometer.Connect()
        if res < 0:
            return res
        self.ConnessoProfilometer = True

        return 0

    def Disconnect(self):
        "Si disconnette dal robot con l'IP impostato nella variabile 'robotIP'"
        if self.Connesso:
            self.robot.close()
            #if not self.useLogger: self.robotLogger.close() #not working anymore (RAPID has changed)
            self.Connesso = False
        if self.ConnessoLuce:
            self.light.close()
            self.ConnessoLuce = False
            print("Light Disconnected")
        if self.ConnessoCamera:
            self.camera.Disconnect()
            self.ConnessoCamera = False
            print("Camera Disconnected")
        if self.ConnessoProfilometer:
            self.profilometer.Disconnect()
            self.ConnessoProfilometer = False
            print("Profilometer Disconnected")

        return 0

    def SetToolLength(self, NewLen : float):
        if not self.Connesso:
            return -1

        self.robot.set_tool([[0, 0, NewLen], [1, 0, 0, 0]]) #La lunghezza del tool è fino al sensore della camera
        self.toolLen = NewLen

        print("Tool length changed to " + str(NewLen))

        return 0

    def SetSingularityInterpolation(self, value):
        '''
        value
            0 -> Off
            1 -> LockAxis4
            2 -> Wrist
        '''
        if not self.Connesso:
            return -1
        if not (value == 0 or value == 1 or value == 2):
            return -2

        self.robot.change_singularity_interp(value)
        print("Singularity changed to " + str(value))

        return 0

    def SetConfigurationLinearMonitoring(self, value):
        '''
        value
            0 -> Off
            1 -> On
        '''
        if not self.Connesso:
            return -1
        if not (value == 0 or value == 1):
            return -2

        self.robot.change_singularity_interp(value)
        print("ConfL changed to " + str(value))

        return 0

    def SetLightIntensity(self, intensity):
        '''
        Set light intensity
        '''
        if not self.ConnessoLuce:
            return -2
        if intensity < 0 or intensity > 255:
            return -1
        self.lightIntensity = intensity
        self.light.setIntensity(self.lightChannel, intensity)
        return 0

    def CaptureFrameFromCamera(self, exposureTime, gainValue, lightIntensity):
        '''
        Acquire a single frame from camera with specified settings
        '''
        if not self.ConnessoCamera:
            return -3
        res = self.SetLightIntensity(lightIntensity)
        if res < 0:
            print("TestCameraLight - Impossible to change light settings")
        img = self.camera.CaptureFrame(exposureTime, gainValue)
        if img is int:
            print("TestCameraLight - Error grabbing frame")
        return img

    def CameraCalibration(self, TargetPoint, TargetNormal, TargetMajorAxis, Distance, AngleXFrom, AngleXTo, AngleXStep, AngleYFrom, AngleYTo, AngleYStep
                          , SaveFolder, ExposureTime, GainRaw, LightIntensity, SaveImages):
        '''
        TargetPoint is the position (robot coordinate) of the center of the image
        TargetNormal is the normal to the surface on which the printed image is attached
        TargetMajorAxis specifies the direction of the major axis of the printed target image
        Distance is the distance from which to take photos
        Angles are Integer, degrees.
        SaveImages is a boolean
        '''

        XC = TargetPoint[0]
        YC = TargetPoint[1]
        ZC = TargetPoint[2]

        res = FrankUtilities.Target3rdVersor2PosRT(TargetPoint, TargetNormal, TargetMajorAxis, Distance)
        if res == -1:
            return -1
        M1 = res[1]
        M1 = FrankCommons.MatrixProduct(M1, self.camera.CameraCoordSystem)

        for xx in range(AngleXFrom, AngleXTo+AngleXStep, AngleXStep):
            for yy in range(AngleYFrom, AngleYTo+AngleYStep, AngleYStep):
                RT = FrankUtilities.Euler2RotMatrix([0, math.radians(yy), math.radians(xx)])
                RT = FrankCommons.MatrixProduct(RT, M1)
                v3 = [RT[0][2], RT[1][2], RT[2][2]]
                PP = [XC - v3[0]*Distance, YC - v3[1]*Distance, ZC - v3[2]*Distance]
                self.GoToCartesianRT(PP,RT)
                time.sleep(2)
                img = self.CaptureFrameFromCamera(ExposureTime, GainRaw, LightIntensity)
                if img is int:
                    print("Image Acquisition Error")
                else:
                    self.myevents.CameraCalib_NewImage(img)
                    if SaveImages:
                        cv2.imwrite(Folder + "IMG_X" + "{:+02d}".format(xx) + "_Y" + "{:+02d}".format(yy) + ".tiff",img)

    def SetSpeed(self, linear: int, orientation: int, external_linear: int, external_orientation: int):
        '''
        speed: [robot TCP linear speed (mm/s), TCP orientation speed (deg/s),
                external axis linear, external axis orientation]
        '''
        if not self.Connesso: return -1
        self.robot.set_speed([linear, orientation, external_linear, external_orientation])

        print("Speed changed to " + str(linear) + " (linear) , " + str(orientation) + " (orientation) , " 
              + str(external_linear) + " (external_linear) , " + str(external_orientation) + " (external_orientation)")

        return 0

    def GetSpeed(self):
        '''
        speed: [robot TCP linear speed (mm/s), TCP orientation speed (deg/s),
                external axis linear, external axis orientation]
        '''
        if not self.Connesso: return -1

        return self.robot.get_speed()

    def GoToJointsCoord(self, J1, J2, J3, J4, J5, J6):
        '''
        Joints coordinate are in degree
        '''
        if not self.Connesso:
            return -1
        self.robot.set_joints([J1, J2, J3, J4, J5, J6])


    def GoToCartesianEuler(self, newPosition, newEZ, newEY, newEX):
        '''
        Cartesian coordinate are in mm, angles in degrees
        '''
        if not self.Connesso:
            return -1
        EZ = math.radians(newEZ)
        EY = math.radians(newEY)
        EX = math.radians(newEX)
        quat = FrankUtilities.Euler2Quaternion([EZ,EY,EX])
        if quat == -1: return -1
        self.robot.set_cartesian([newPosition, quat])

    def GoToCartesianRT(self, newPosition, RTMatrix):
        '''
        Cartesian coordinate are in mm
        '''
        if not self.Connesso:
            return -1
        quat = FrankUtilities.RotMatrix2Quaternion(RTMatrix)
        if quat == -1: return -1
        self.robot.set_cartesian([newPosition,quat])

    def WorkingAreaGetPhotos(self, P1, P2, P3, P4, MaximumDistance, ExposureTime, GainRaw, LightIntensity, SaveFolder, SaveToFile):
        '''
        Acquire Working Area specified by 4 vertexes of a square.
        P1, P2, P3, P4 are the coordinate of the four vertexes (pay attention to normal versor to the surface used as Normal).
        MaximumDistance is the maximum distance from rectangle plane.
        Exposure, Gain and LightIntensity are defined for camera and light
        SaveFolder is the folder to save the photos (with final "/").
        SaveToFile is a Boolean.
        ...
        Returns an error code.
        '''
        points = self.camera.GetStitchingPointsFromRectVertexes(P1, P2, P3, P4, MaximumDistance)
        if points == -1: return -1
        points = self.camera.AcquisitionPoints
        RT = self.camera.AcquisitionRT
        LocalPoints = self.camera.ImagesCenterPos

        N = len(points)

        if SaveToFile:
            #Write text file with informations
            file = open(SaveFolder + "info.txt","w",1)
            file.write("Stitching points calculation\n")
            file.write("\n")
            file.write("Camera Sensor Name: " + self.camera.SensorName + "\n")
            file.write("Camera Sensor Width: " + str(self.camera.SensorWidth) + "\n")
            file.write("Camera Sensor Height: " + str(self.camera.SensorHeight) + "\n")
            file.write("Camera Sensor Focal Length: " + str(self.camera.FocalLength) + "\n")
            file.write("Image Border Increase: " + str(self.camera.ImageBorder) + "\n")
            file.write("Image Minimum Overlap: " + str(self.camera.ImageOverlap) + "\n")
            file.write("\n")
            file.write("Stitching required: " + str(self.camera.StitchingRequired) + "\n")
            file.write("Number of photos: " + str(self.camera.NumImages) + "\n")
            file.write("Photos along X: " + str(self.camera.NumHImages) + "\n")
            file.write("Photos along Y: " + str(self.camera.NumVImages) + "\n")
            file.write("Effective Distance: " + str(self.camera.Distance) + "\n")
            file.write("Effective Field Of View: " + str(self.camera.EffectiveFieldOfView) + "\n")
            file.write("\n")
            file.write("Exposure Time: " + str(ExposureTime) + "\n")
            file.write("Gain Raw: " + str(GainRaw) + "\n")
            file.write("Light Intensity: " + str(LightIntensity) + "\n")
            file.write("\n")
            file.write("- Conversion Rectangle to Robot coordinate -\n")
            rectOffset = self.camera.rectToRobotOffset
            rectEul = FrankUtilities.RotMatrix2Euler(self.camera.rectToRobotRT)
            fstr = "{0:.4f}"
            file.write(fstr.format(rectOffset[0]) + "\t" + fstr.format(rectOffset[1]) + "\t" + fstr.format(rectOffset[2])
                       + "\t" + str(math.degrees(rectEul[0])) + "\t" + str(math.degrees(rectEul[1])) + "\t" 
                       + str(math.degrees(rectEul[2])) + "\n")
            file.write("\n")
            file.write("- Points Local Coordinate -\n")
            for i in range(0, N):
                file.write(fstr.format(LocalPoints[i][0]) + "\t" + fstr.format(LocalPoints[i][1]) + "\t" 
                           + fstr.format(LocalPoints[i][2]) + "\n")
            file.write("\n")
            file.write("- Points Robot Coordinate and Euler Angles (EZ, EY, EX) -\n")
            for i in range(0, N):
                eul = FrankUtilities.RotMatrix2Euler(RT[i])
                file.write(fstr.format(points[i][0]) + "\t" + fstr.format(points[i][1]) + "\t" + fstr.format(points[i][2]) 
                           + "\t" + str(math.degrees(eul[0])) + "\t" + str(math.degrees(eul[1])) + "\t" 
                           + str(math.degrees(eul[2])) + "\n")
            file.flush()
            file.close()

        fstr = "{:0" + str(len(str(N))) + "d}"

        self.robot.set_zone(point_motion = True)
        time.sleep(0.5)

        for i in range(0, N):
            quat = FrankUtilities.RotMatrix2Quaternion(RT[i])
            if quat == -1: 
                self.robot.set_zone()
                return -1
            self.robot.set_cartesian([points[i], quat])
            time.sleep(0.2)
            img = self.CaptureFrameFromCamera(ExposureTime, GainRaw, LightIntensity)
            if img is int:
                self.robot.set_zone()
                return -1
            else:
                self.myevents.WorkingArea_NewImage(img)
                if SaveToFile:
                    cv2.imwrite(SaveFolder + fstr.format(i+1) + ".tiff", img)

        self.robot.set_zone()

        return 0

    def ConfigureStitcher(self, px4mm, angle, RotCenterX, RotCenterY, pixelCrop):

        self.stitcher = FrankStitcher.newStitcher(px4mm, angle, RotCenterX, RotCenterY, pixelCrop)

    def StitchImages(self, FolderPath, SaveOutput, UseCamera):
        if UseCamera:
            return self.stitcher.StitchImages(FolderPath, SaveOutput, self.camera)
        else:
            return self.stitcher.StitchImages(FolderPath, SaveOutput)

    def EdgeDetection(self, FilePath, CannyThreshold1, CannyThreshold2, BrightNessThreshold, SaveToFile):
        if self.stitcher == None:
            print("Stitcher not initialized")
            return None
            
        return self.stitcher.EdgeRecognition(FilePath, CannyThreshold1, CannyThreshold2, BrightNessThreshold, SaveToFile)

    def AcquireMultiProfile(self, feedrate, StartCoord, StartRT, EndCoord, EndRT, SavePath, SaveInputToFile, SaveToFile):

        t1 = threading.Thread(target = self.__AcquireMultiProfileThreadHelper
                              , args=(feedrate, StartCoord, StartRT, EndCoord, EndRT, SavePath, SaveInputToFile, SaveToFile,))
        t1.start()

        pass
    def AcquireMultiProfileThreadHelper(self, feedrate, StartCoord, StartRT, EndCoord, EndRT, SavePath , SaveInputToFile, SaveToFile):
        
        self.__AcquireMultiProfileThreadHelper(feedrate, StartCoord, StartRT, EndCoord, EndRT, SavePath , SaveInputToFile, SaveToFile)
    
    def __AcquireMultiProfileThreadHelper(self, feedrate, StartCoord, StartRT, EndCoord, EndRT, SavePath , SaveInputToFile, SaveToFile):
        '''
                                         
        feedrate - cartesian feed in mm per seconds
        G0feed - feed to move from one point to the other
        StartCoord - starting point in robot coordinate (list)
        EndCoord - ending point in robot coordinate (list)
        '''

        N = len(StartCoord) #StartRT, EndCoord and EndRT must have the same length
        self.MultiXX = []
        self.MultiYY = []
        self.MultiProfile = []
        self.MultirC = []
        self.num = 0

        pS = self.GetSpeed()

        self.SetLightIntensity(0)

        self.robot.set_zone(point_motion = True)

        for i in range (N):

            self.CoordinateLogSaveToFile = SaveInputToFile #SaveToFile
            self.CoordinateLogFileName = SavePath + "/Coords" + str(i) + ".txt"

            res = self.SetSpeed(pS[0], pS[1], pS[2], pS[3])
            if res < 0: return res
            self.GoToCartesianRT(StartCoord[i], StartRT[i])
            time.sleep(0.2)

            res = self.SetSpeed(feedrate, pS[1], pS[2], pS[3])
            if res < 0: return res
            time.sleep(0.2)

            t3 = threading.Thread(target = self.__ProfilometerMultiAcquireThreadHelper, args=(feedrate, SavePath, SaveInputToFile,))
            t3.start()

            while self.profilometer.TriggerStarted == False:
                time.sleep(0.1)
                
            print ("Goto End point Requested")
            self.GoToCartesianRT(EndCoord[i], EndRT[i])
            print ("Goto End point done")
            time.sleep(0.2)

            self.profilometer.RequestedStop = True

            t3.join()
            
            pass

        self.robot.set_zone() #returns to default
        res = self.SetSpeed(pS[0], pS[1], pS[2], pS[3])

        if SaveToFile:
            #time.sleep(1)
            self.profilometer.Build_RobotMultiPointCloud(self.MultirC, self.MultiXX, self.MultiYY, self.MultiProfile, SavePath + "/PointCloud.ply")
            print("PLY file saved")
        
        pass

    def AcquireClusterProfileThreadHelper(self, feedrate, Coord_list, Pose_list, SavePath , SaveInputToFile, SaveToFile):
        self.MultiXX = []
        self.MultiYY = []
        self.MultiProfile = []
        self.MultirC = []
        self.num = 0

        pS = self.GetSpeed()

        self.SetLightIntensity(0)

        self.robot.set_zone(point_motion = True)
        self.CoordinateLogSaveToFile = SaveInputToFile #SaveToFile
        self.CoordinateLogFileName = SavePath + "/Coords.txt"

        res = self.SetSpeed(pS[0], pS[1], pS[2], pS[3])
        if res < 0: return res
        self.GoToCartesianRT(Coord_list[0], Pose_list[0])      # va al primo punto
        time.sleep(0.2)

        res = self.SetSpeed(feedrate, pS[1], pS[2], pS[3])
        if res < 0: return res
        time.sleep(0.2)

        t3 = threading.Thread(target = self.__ProfilometerMultiAcquireThreadHelper, args=(feedrate, SavePath, SaveInputToFile,))
        t3.start()

        while self.profilometer.TriggerStarted == False:
            time.sleep(0.1)
        print ("Goto End point Requested")
        for i in range(len(Coord_list)):      
            self.GoToCartesianRT(Coord_list[i], Pose_list[i])
            time.sleep(0.2)

        print ("Goto End point done")
        time.sleep(0.2)

        self.profilometer.RequestedStop = True

        t3.join()
            
        pass

        self.robot.set_zone() #returns to default
        res = self.SetSpeed(pS[0], pS[1], pS[2], pS[3])

        if SaveToFile:
            #time.sleep(1)
            self.profilometer.Build_RobotMultiPointCloud(self.MultirC, self.MultiXX, self.MultiYY, self.MultiProfile, SavePath + "/PointCloud.ply")
            print("PLY file saved")
        
        pass



    MultiXX = []
    MultiYY = []
    MultiProfile = []
    MultirC = []
    num = 0

    def __ProfilometerMultiAcquireThreadHelper(self, feed, SavePath, SaveToFile):
        print("AcquireProfile - ThreadHelper Started ")
        res = self.profilometer.Acquire(self.robot, self.LoggerSampleTime, SavePath + "/PointCloud_" + str(self.num) 
                                        + ".txt", SaveToFile, RobotLinSpeed = feed)
        self.num = self.num + 1
        print("AcquireProfile - ThreadHelper Acquisition Completed ")
        if type(res) is int:
            if res < 0:
                return res
            pass

        rC = self.CoordinateLog
        [XX, YY, Profile] = res

        self.MultiXX.append(XX)
        self.MultiYY.append(YY)
        self.MultiProfile.append(Profile)
        self.MultirC.append(rC)

        return 0
