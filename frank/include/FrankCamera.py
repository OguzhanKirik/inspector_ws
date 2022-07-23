import math
import sys
from pathlib import Path
include_dir = str(Path(__file__).resolve().parent)
sys.path.append(include_dir)
utils_dir = str(Path(__file__).resolve().parent.parent.parent)
sys.path.append(utils_dir)
from UTILS import FrankCommons
from UTILS import FrankUtilities
import cv2
#Required for Camera acquisition
from pypylon import pylon
import numpy as np
#Required for Camera acquisition

class newCamera():

    #Camera data *********************
    SensorName = "Fujifilm HF2518-12M" #https://visionlink.it/it/obiettivi-machine-vision/1993-hf2518-12m.html
    SensorWidth = 7.6 # mm
    SensorHeight = 5.7 # mm
    FocalLength = 25 # mm
    CameraCoordSystem = [[0, 1, 0],
                        [-1, 0, 0],
                         [0, 0, 1]] #(tool Coordinate) Z Versor exits from camera lens. X Versor is directed parallel to SensorWidth.
    #*********************************

    #Stitching data ******************
    ImageOverlap = 0.5  # Overlap percentage between adiacent images
    ImageBorder = 0.05  # Percentage of border around FieldOfView (if the target is 1000x1000 mm, the area to be captured is 1100x1100 (both sides))
    #*********************************

    #Images to acquire ***************
    StitchingRequired = False
    Distance = 0.0
    EffectiveFieldOfView = [0.0, 0.0]
    NumImages = 0
    NumHImages = 0
    NumVImages = 0
    ImagesCenterPos = [] #Coordinate Locali
    AcquisitionPoints = [] #Coordinate Robot
    AcquisitionRT = []
    rectToRobotOffset = []
    rectToRobotRT = []
    #*********************************

    #*********************************
    camera = None
    Connected = False
    ExposureTime = 300000
    GainValue = 1500
    #*********************************

    def __init__(self, SensorName = "Fujifilm HF2518-12M", SensorWidth = 7.6, SensorHeight = 5.7, FocalLength = 25
                 , CameraCoordSystem =  [[0, 1, 0],[-1, 0, 0],[0, 0, 1]], ImageOverlap = 0.5, ImageBorder = 0.05):
        '''
        CameraCoordSystem: (tool Coordinate) Z Versor exits from camera lens. X Versor is directed parallel to SensorWidth.
        ImageOverlap: Overlap percentage between adiacent images
        ImageBorder: Percentage of border around FieldOfView (if the target is 1000x1000 mm, the area to be captured is 1100x1100 (both sides))
        '''
        self.SensorName = SensorName
        self.SensorWidth = SensorWidth
        self.SensorHeight = SensorHeight
        self.FocalLength = FocalLength
        self.CameraCoordSystem = CameraCoordSystem
        self.ImageOverlap = ImageOverlap
        self.ImageBorder = ImageBorder


    def CalculateWorkingDistance(self, FieldOfView):
        '''
        Calculate the minimum working distance to frame the specified Field Of View.
        La distanza è calcolata dal sensore"
        Return -1 if an error is present
        '''
        res = FrankCommons.Check_Array(FieldOfView)
        if res != 2:
            print("CalculateWorkingDistance, input error")
            return -1

        Horizontal = FieldOfView[0]
        Vertical = FieldOfView[1]

        if Horizontal <= 0 or Vertical <= 0:
            return -1

        dist1 = self.FocalLength / self.SensorWidth * Horizontal
        dist2 = self.FocalLength / self.SensorHeight * Vertical

        return max(dist1, dist2) + self.FocalLength

    def CalculateFieldOfView(self, Distance):
        '''
        Calculate di dimension of the Field of View [Horizontal, Vertical].
        Distance is from the sensor, not from the Focal Point.
        Return -1 if an error is present
        '''

        if Distance <= 0:
            return -1

        ww = self.SensorWidth / self.FocalLength * (Distance - self.FocalLength)
        hh = self.SensorHeight / self.FocalLength * (Distance - self.FocalLength)

        return [ww, hh]

    def ImageStitchingRequired(self, FieldOfView, MaximumDistance):
        '''
        Input:
        1. FieldOfView is a List of 2 elements: horizontal dimension (width) and vertical dimension (height).
        2. MaximumDistance specify the maximum distance that is possible to reach with the camera handling device.
        ...
        Return -1 if an error is present
        Without errors returns an array: [IsRequired, Distance, FOV, NumImages, ImagesCenterPos]
        ...
        Output:
        1. IsRequired - If Stitching is required or not
        2. Distance - The distance between camera sensor and (plane) object
        3. FOV - Effective Field Of View
        4. NumImages - Number of required images
        5. ImagesCenterPos - List of X-Y-Z coordinare with Zero in the Top-Left of the rectangle (in fact in any corner with positive signs directed to the opposite corner along a side). Z is always zero
        '''
        res = FrankCommons.Check_Array(FieldOfView)
        if res != 2:
            print("ImageStitchingRequired, input error")
            return -1

        num = 1 + self.ImageBorder * 2 # 2 times because is applied on all sides of the rectangle
        Target = [FieldOfView[0] * num, FieldOfView[1] * num]

        Distance = self.CalculateWorkingDistance(Target)
        if Distance == -1:
            print("ImageStitchingRequired, CalculateWorkingDistance error")
            return Distance

        #Checks if the required distance from the sensor is higher than the maximum reacheable one
        IsRequired = Distance > MaximumDistance

        Horizontal = Target[0]
        Vertical = Target[1]

        NumImages = 1
        HImages = 1
        VImages = 1
        FOV = Target
        ImagesCenterPos = [[Horizontal / 2 - self.ImageBorder * FieldOfView[0], Vertical / 2 - self.ImageBorder * FieldOfView[1]], Distance]
    
        if IsRequired:
            FOV = self.CalculateFieldOfView(MaximumDistance) # Maximum Field of View based on Maximum Distance
            HImages = (Horizontal / FOV[0] - self.ImageOverlap) / (1 - self.ImageOverlap) # Derives from a percentege overlap of images
            VImages = (Vertical / FOV[1] - self.ImageOverlap) / (1 - self.ImageOverlap) # Derives from a percentege overlap of images
            HImages = math.ceil(HImages)
            VImages = math.ceil(VImages)
            NumImages = HImages * VImages
            Distance = MaximumDistance
            ImagesCPos = [0.0]*NumImages
            dX = (Horizontal - FOV[0]) / (HImages - 1) # Overlap percentage changes (minimum is ImageOverlap)
            dY = (Vertical - FOV[1]) / (VImages - 1) # Different from the one in Horizontal direction
            qX = FOV[0] / 2 - self.ImageBorder * FieldOfView[0]
            qY = FOV[1] / 2 - self.ImageBorder * FieldOfView[1]
            for i in range(0, HImages):
                for j in range(0, VImages):
                    ImagesCPos[i + j * HImages] = [0.0, 0.0, 0.0]
                    if j % 2 == 0:
                        ImagesCPos[i + j * HImages][0] = dX * i + qX
                    else:
                        ImagesCPos[i + j * HImages][0] = dX * (HImages - i - 1) + qX
                    ImagesCPos[i + j * HImages][1] = dY * j + qY
                    ImagesCPos[i + j * HImages][2] = Distance

        self.StitchingRequired = IsRequired
        self.Distance = Distance
        self.EffectiveFieldOfView = FOV
        self.NumImages = NumImages
        self.NumHImages = HImages
        self.NumVImages = VImages
        self.ImagesCenterPos = ImagesCPos

        return 0 #No Error #[IsRequired, Distance, FOV, NumImages, ImagesCenterPos]

    def GetStitchingPointsFromRectVertexes(self, P1, P2, P3, P4, MaximumDistance):
        '''
        Returns the robot coordinate positions and rotation in which to take pictures for the Stitching.
        ...
        Output:
        1. List of points in robot coordinate system
        2. For each point, a Rotation matrix
        '''
        rect = GetRectFrom4Vertexes(P1, P2, P3, P4) #Correct Order
        #rect = GetRectFrom4Vertexes(P2, P1, P3, P4) #Wrong Order
        #rect = GetRectFrom4Vertexes(P3, P2, P1, P4) #Correct order but opposite direction with respect to first case
        #rect = GetRectFrom4Vertexes(P4, P2, P3, P1) #Wrong Order
        #rect = GetRectFrom4Vertexes(P1, P3, P2, P4) #Wrong Order
        #rect = GetRectFrom4Vertexes(P1, P4, P3, P2) #Correct order but opposite direction with respect to first case
        #rect = GetRectFrom4Vertexes(P1, P2, P4, P3) #Wrong Order
        if rect == -1: return -1

        [rectSize, rectOffset, rectRT] = rect #X camera is aligned with the first versor of this coordinate system

        dist = self.ImageStitchingRequired(rectSize, MaximumDistance)
        if dist == -1: return -1

        vers = FrankCommons.RotMatrixToVersors(rectRT)
        if vers == -1: return -1

        PosRT = FrankUtilities.Target3rdVersor2PosRT(rectOffset, vers[2], vers[0], 0)
        if PosRT == -1: return -1

        RT = FrankCommons.MatrixProduct(PosRT[1], self.CameraCoordSystem)
        if RT == -1: return -1

        points = FrankCommons.ChangeCoordsToAbsolute(self.ImagesCenterPos, rectOffset, rectRT)
        if points == -1: return -1
    
        RTs = [0]*len(points)
        for i in range(0, len(points)):
            RTs[i] = RT

        self.AcquisitionPoints = points
        self.AcquisitionRT = RTs
        self.rectToRobotOffset = rectOffset
        self.rectToRobotRT = rectRT
        return 0 #No Error #[points, RTs]

    def Connect(self):
        if self.Connected:
            return 0
        print("Connecting to camera")
        try:
            self.camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())
            
            self.camera.Open()
            self.Connected = True
                    #
            self.camera.ExposureAuto.SetValue('Off')
            self.camera.GainAuto.SetValue('Off')

            self.camera.ExposureTimeAbs.SetValue(self.ExposureTime) #microsecondi
            self.camera.GainRaw.SetValue(self.GainValue)
            self.camera.GevSCPSPacketSize.SetValue(1500)
        
            self.camera.AcquisitionMode.SetValue('SingleFrame')

                    #
            print("Camera connected")
            pass
        except :
            print("Camera connection error")
            self.Connected = False
            pass
        
        if self.Connected:
            return 0
        else:
            return -4

    def Disconnect(self):
        if self.Connected:
            try:
                self.camera.Close()
                return 1
                pass
            except :
                return 0
                pass
        else:
            return -1
    def CaptureFrame(self, ExposureTime, GainValue):
        '''
        Acquire an image from the camera on the tool of the robot, returns -1 in case of error
        Exposure Time is in microseconds
        Gain (Raw) value is a integer
        '''
        if self.Connected:
            self.ExposureTime = ExposureTime
            self.GainValue = GainValue
            
            #camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())
            #camera.Open()
            
            # self.camera.ExposureAuto.SetValue('Off')
            # self.camera.GainAuto.SetValue('Off')

            # self.camera.ExposureTimeAbs.SetValue(ExposureTime) #microsecondi
            # self.camera.GainRaw.SetValue(GainValue)
            
            # self.camera.GevSCPSPacketSize.SetValue(1500)
            
            # self.camera.AcquisitionMode.SetValue('SingleFrame')

            self.camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)

            #cv2.namedWindow('Camera', cv2.WINDOW_AUTOSIZE)

            grabResult = self.camera.RetrieveResult(10000, pylon.TimeoutHandling_ThrowException)
            #img = grabResult.Array
            success = grabResult.GrabSucceeded()
            converter = pylon.ImageFormatConverter()
            converter.OutputPixelFormat = pylon.PixelType_BGR8packed
            converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned
            image = converter.Convert(grabResult)
            img = image.GetArray()
            # cv2.namedWindow('foto', cv2.WINDOW_NORMAL)
            # cv2.imshow('foto', img)
            # k = cv2.waitKeyEx()
    
            # if grabResult.GrabSucceeded():
            #     cv2.imshow('Camera',cv2.resize(img,(1006,759)))
            grabResult.Release()

            self.camera.StopGrabbing()
        # cv2.destroyAllWindows()
            if success:
                return img
            else:
                return -1
        else:
            return -1

def ThreePointsToNormal(P1, P2, P3):
    "Calculate the normal vector (plane) from the three specified points"
    n = FrankCommons.Check_Array(P1)
    if n != 3:
        print("GetRectFromPoints, input error")
        return -1
    n = FrankCommons.Check_Array(P2)
    if n != 3:
        print("GetRectFromPoints, input error")
        return -1
    n = FrankCommons.Check_Array(P3)
    if n != 3:
        print("GetRectFromPoints, input error")
        return -1

        # Calculate the normal to the plane described by P1-P2-P3
    V1 = [P2[0]-P1[0], P2[1]-P1[1], P2[2]-P1[2]]
    V2 = [P3[0]-P2[0], P3[1]-P2[1], P3[2]-P2[2]]
    n1 = FrankCommons.crossProduct(V1, V2)

    return n1

def GetRectFrom4Vertexes(P1, P2, P3, P4):
    '''
    Calculate the dimension of the rectangle to be photographed starting from the four vertexes in a 3D space
    Returns:
    1. Size of the rectangle: "Horizontal" and "Vertical"
    2. Offset of the reference system (P1)
    3. Rotation Matrix of the new reference system
    '''

    n = FrankCommons.Check_Array(P1)
    if n != 3:
        print("GetRectFromPoints, input error")
        return -1
    n = FrankCommons.Check_Array(P2)
    if n != 3:
        print("GetRectFromPoints, input error")
        return -1
    n = FrankCommons.Check_Array(P3)
    if n != 3:
        print("GetRectFromPoints, input error")
        return -1
    n = FrankCommons.Check_Array(P4)
    if n != 3:
        print("GetRectFromPoints, input error")
        return -1

    # Calculate the normal to the plane described by P1-P2-P3
    n1 = ThreePointsToNormal(P1, P2, P3)

    # Calculate the normal to the plane described by P2-P3-P4
    n2 = ThreePointsToNormal(P2, P3, P4)

    # Calculate the normal to the plane described by P1-P2-P4
    n3 = ThreePointsToNormal(P1, P2, P4)

    # Calculate the normal to the plane described by P1-P3-P4
    #n4 = ThreePointsToNormal(P1, P3, P4) # Not needed, see below

    # If all four normals are equal the sequence of the points is correct (clockwise or counter-clockwise)

    # If the four points lie on the same plane the cross product is zero. Here the directions of the vectors are checked
    n1n2 = True
    n1n3 = True
    #n1n4 = True # Not needed, see below
    #n2n3 = True # Not needed, see below
    #n2n4 = True # Not needed, see below
    #n3n4 = True # Not needed, see below
    # Reverse Sequence
    #  P1-P2   FTFFTF
    #  P1-P3   TTTTTT (opposite sign with respect to correct sequence P1-P2-P3-P4)
    #  P1-P4   TFFFFT
    #  P2-P3   TFFFFT
    #  P2-P4   TTTTTT (opposite sign with respect to correct sequence P1-P2-P3-P4)
    #  P3-P4   FTFFTF
    # It is sufficient to check n1n2 and n1n3
    check = FrankCommons.crossProduct(n1, n2)
    for i in range(0,3):
        n1n2 = n1n2 and (abs(n1[i] - n2[i]) < FrankCommons.tol) # If points are in the correct sequence, the directions are the same
        n1n3 = n1n3 and (abs(n1[i] - n3[i]) < FrankCommons.tol) # If points are in the correct sequence, the directions are the same
        #n1n4 = n1n4 and ((n1[i] - n4[i]) < FrankCommons.tol) # If points are in the correct sequence, the directions are the same
        #n2n3 = n2n3 and ((n2[i] - n3[i]) < FrankCommons.tol) # If points are in the correct sequence, the directions are the same
        #n2n4 = n2n4 and ((n2[i] - n4[i]) < FrankCommons.tol) # If points are in the correct sequence, the directions are the same
        #n3n4 = n3n4 and ((n3[i] - n4[i]) < FrankCommons.tol) # If points are in the correct sequence, the directions are the same
        if (abs(check[i]) > FrankCommons.tol):
            print("GetRectFromPoints, Points are not on the same plane")
            return -1
    check = True
    sequence = n1n2 and n1n3
    if sequence == False:
        print("GetRectFromPoints, point sequence is not a correct, check normal vectors of triangles")
        return -1
    # ******************************************************************

    # Here the four points are on the same plane and the sequence P1-P2-P3-P4 is either clockwise or counter-clockwise
    
    # Now check if is a rectangle **************************************
    rectangle = False
    AB = FrankCommons.SumOfSquaresDifference(P1, P2)
    BC = FrankCommons.SumOfSquaresDifference(P2, P3)
    CD = FrankCommons.SumOfSquaresDifference(P3, P4)
    DA = FrankCommons.SumOfSquaresDifference(P4, P1)
    AC = FrankCommons.SumOfSquaresDifference(P1, P3)

    if abs(AB - CD) > FrankCommons.tol:
        print("GetRectFromPoints, Points do not form a rectangle")
        return -1
    if abs(BC - DA) > FrankCommons.tol:
        print("GetRectFromPoints, Points do not form a rectangle")
        return -1
    if abs(AC - (AB + BC)) > FrankCommons.tol:
        print("GetRectFromPoints, Points do not form a rectangle")
        return -1
    rectangle = True
    # ******************************************************************

    # Here the four points represents a rectangle

    # Calculate a local reference system *******************************
    AB = math.sqrt(AB) #Length of side AB
    BC = math.sqrt(BC) #Length of side BC
    DA = math.sqrt(DA) #Length of side DA

    offset = P1
    v1 = [0, 0, 0]
    v1[0] = (P2[0] - P1[0]) / AB
    v1[1] = (P2[1] - P1[1]) / AB
    v1[2] = (P2[2] - P1[2]) / AB
    v2 = [0, 0, 0]
    v2[0] = (P4[0] - P1[0]) / DA
    v2[1] = (P4[1] - P1[1]) / DA
    v2[2] = (P4[2] - P1[2]) / DA

    v3 = FrankCommons.crossProduct(v1, v2)
    RT = FrankCommons.VersorsToRotMatrix(v1, v2, v3)
    # ******************************************************************

    return [[AB, BC], offset, RT]
