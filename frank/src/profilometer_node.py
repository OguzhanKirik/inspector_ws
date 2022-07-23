#! /usr/bin/env python3
from posixpath import expanduser
import sysconfig

A = sysconfig.get_config_vars("LIBDIR","INSTSONAME")
print(A)
import rospy
import sys
import pathlib
from os.path import dirname, abspath

# include path python
include_dir = str(dirname(dirname(abspath(__file__)))) + '/include/'
sys.path.append(include_dir)
import FrankProfilometer
from std_msgs.msg import String, Bool, Int32, Int16
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint
import json
#import pcl
import abb
import numpy as np
import time
from frank.msg import profiles_msg, OperationState
from  frank.srv import *
import threading 
def append_profiles(msg):
        if Profi.Append ==True:
            Profi.pose_robot.append([msg.transforms[0].translation.x,
                                    msg.transforms[0].translation.y,
                                    msg.transforms[0].translation.z,
                                    msg.transforms[0].rotation.w,
                                    msg.transforms[0].rotation.x,
                                    msg.transforms[0].rotation.y,
                                    msg.transforms[0].rotation.z,
                                     msg.time_from_start.to_sec(),msg.time_from_start.to_nsec() ])
def trigger_state_callback(msg):
    if msg.data ==1:
        Profi.TriggerStarted == True
    if msg.data ==0:
        Profi.TriggerStarted==False

def make_profiles_msg(XX, YY, ZZ):
    msg = profiles_msg()
    msg.XX = [XX]
    msg.YY = [YY]
    msg.ZZ = [ZZ]
    return msg

def SaveProfilo(profilo, xscale, savepath):
    numX = len(profilo)*xscale
    XX  = np.arange(-numX/2, numX/2, xscale)
    YY = np.zeros(len(profilo))
    file = open(savepath+'data.txt',"w")
    stringlist = '\t'.join([str(item) for item in XX]) + '\n'
    file.write(stringlist)
    stringlist = '\t'.join([str(item) for item in YY]) + '\n'
    file.write(stringlist)
    stringlist = '\t'.join([str(item) for item in profilo]) + '\n'
    file.write(stringlist)
    file.close()
    print("profilo salvato")



def connect_profilometer(req):
    response = ConnectProfilometerResponse()
    if Profi.Connected:
        print("profilometer is already connected")
        response.error = "profilometer is already connected"
        response.res = 1
        return response
    else:
        print("...Connecting to Profilometer...")
        res=-1
        count = 0
        while res<0 or count<4:
            res = Profi.Connect()
            time.sleep(1)
            count+=1
        if res <0:
            print("error connecting to profilometer")

            response.res = 0
            response.error = "error connecting to profilometer"
            return response
        else:
            print("profilometer connected successfully")
            response.res = 1
            response.error = "connected successfully to profilometer"
            return response

def disconnect_profilometer(req):
    response = DisconnectProfilometerResponse()
    if Profi.Connected==False:
        print("profilometer is already disconnected")
        response.error = "profilometer is already disconnected"
        response.res = 1
        return response
    else:
        print("...Disconnecting to Profilometer...")
        res = Profi.Disconnect()
        if res<0:
            print("error disconnecting to profilometer")
            response.res = 0
            response.error = "error disconnecting to profilometer"
            return response
        else:
            print("profilometer disconnected successfully")
            response.res = 1
            response.error = "disconnected successfully to profilometer"
            return response


def trigger_callback(msg):
    if msg.data == 1:
        Profi.TriggerStarted = True
        return
    if msg.data == 0:
        Profi.TriggerStarted = False
        return
def Acquire_trigger_thread(trig_starter,statepub, savepath, savetofile):
    result = Profi.Acquire_trigger(trig_starter,statepub, SavePath=savepath, SaveToFile = savetofile)  #will wait for the profilometer data result

def start_acquisition(req):
    response = StartProfiResponse()
    if Profi.Connected ==False:
        print("profilometer is not connected, cannot start acquisition")
        response.res = 2
        response.error = "proiflometer is not connected"
        return response
    if Profi.AcquisitionStarted ==True:
        print("profilometer is already acquiring , cannot start acquisition")
        response.res = 3
        response.error = "proiflometer is already acquiring"
        return response
    else:
        print("...Starting Profilometer Acquisition...")
        #trigger_pub = rospy.Publisher("start_trigger",Int32, queue_size=1)
        Profi.firstDense = True
        Profi.id_acqu = req.label
        Profi.Filepath=str(rospy.get_param("FilePath/dense"))
        print("file path from get_param = ", Profi.Filepath)
        savepath = Profi.Filepath
        savetofile = True
        t1 = threading.Thread(target = Acquire_trigger_thread, args = (trig_starter,statepub, savepath, savetofile))
        t1.start() # start profilometer setup and acquisition function, service return response if it is ready
        print("waiting for profi to be ready")
        while Profi.ProfiReady==False:
            a=0
        response.res = 1
        response.error = "profilometer started successfully"
        return response

def stop_acquisition(req):
    response = StopProfiResponse()
    if Profi.AcquisitionStarted == False:
        print("acquisition is already stopped")
        response.res = 2
        response.error  ="acquisition is alreeady stopped"
        return response
    else:
        print("stopping acquisition")
        Profi.RequestedStop = True
        response.res = 1
        response.error = "acquisition terminated"
        return response
# def current_state(msg):
#     #when a new procedure requested
#     print("current state requested is : ",msg.data)
#     Profi.id_acqu=0
#     if msg.data == 0:
#         #dense scan case
#         Profi.Filepath = "/home/c301/Desktop/TEMP/DENSE_SCAN/"
#     if msg.data ==-5:
#         Profi.Filepath = "/home/c301/Desktop/TEMP/CALIB_PROFI/"
# def initialize_profi(req):
#      response = InitProfiResponse()
#      #Profi.id_acqu =0 #restart counter
#      try:
#          if req.procedure=="dense":
#              Profi.FilePath=rospy.get_param("FilePath/dense")
#          elif req.procedure=="calib":
#              Profi.FilePath=rospy.get_param("FilePath/calib_prof")
#          response.res = 1
#          response.error = "profilometer initialized with success"
#      except:
#          response.res = 0
#          response.error = "unexpected error initializing profilometer"    
def request_save(req):
    res = RequestSaveProfiResponse()
    Profi.Append = True
    Profi.id_acqu = req.label
    Profi.num_profile = 0
    Profi.RequestSave = True
    Profi.AlreadySaved=False
    #response = RequestSaveProfiResponse()
    while Profi.EnableSave == False:
        #wait for profilometer saving to be ready
        a=0
    res.res = 1
    
    return res 
def request_stop_save(req):
    res = RequestStopSaveProfiResponse()
    Profi.RequestStopSave = True
   # Profi.Append = False
    while Profi.EnableSave == True:
        a=0
        #wait for profilometer to stop saving
    # return 0
    res.res = 1
    
    return res 
def enable_save(req):
    Profi.EnableSave = True

if __name__ == '__main__':

    try:
        rospy.init_node('profilometer')
        Profi = FrankProfilometer.newProfilometer()
        #trigger_sub = rospy.Subscriber("trigger_State", Int32, trigger_callback)
        s_conn = rospy.Service("connect_profilometer", ConnectProfilometer, connect_profilometer)
        s_disc = rospy.Service("disconnect_profilometer", DisconnectProfilometer, disconnect_profilometer)
        s_acqu = rospy.Service("start_profi_acquisition", StartProfi,start_acquisition)
        s_stop = rospy.Service("stop_profi_acquisition", StopProfi, stop_acquisition)
        s_req_save = rospy.Service("request_save_profi", RequestSaveProfi, request_save )
        s_req_stop_save = rospy.Service("request_stop_save_profi", RequestStopSaveProfi, request_stop_save )
        #s_enable_save = rospy.Service("enable_save_profi", EnableSaveProfi, enable_save)
        pose_sub = rospy.Subscriber("/robot_feedback",MultiDOFJointTrajectoryPoint, append_profiles)
        #state_sub = rospy.Subscriber("/state_request",Int16, current_state)
        #s_init = rospy.Service("init_profi", InitProfi,initialize_profi)
        trigger_sub = rospy.Subscriber("trigger_state",Int32, trigger_callback)
        trig_starter = rospy.ServiceProxy("plc_command",CommandPLC)
        statepub = rospy.Publisher("/operation_state", OperationState, queue_size =1)
        print("PROFILOMETER is ready to serve")
        # while  not rospy.is_shutdown():
        #     Profi.procedure_case = 


        rospy.spin()
    except  rospy.ROSInterruptException:
        pass




# rospy.init_node('profilometer')
# #sub_connection =  rospy.Subscriber('/profilometer_connection', String, callback)
# rate = rospy.Rate(10)    # 3Hz
# Connected = False
# profi= FrankProfilometer.newProfilometer() 
# while not rospy.is_shutdown():
#     # Waits for connection message
#     msg = rospy.wait_for_message('/profilometer_connection', Bool)
    
#     if msg.data == True:
#         print('provo a connettermi')
#         profi.Connect()
#         print('mi son connesso')
#         msg2 = rospy.wait_for_message('/profilometer_acquisition', String)
#         if msg2.data == True:
#             print('starting acquisition')
#             #robot = abb.Robot(ip = "192.168.125.1")         #is just used to start the triggering command to controller, change with the new command and change Acquire()
#             SavePath = include_dir
#             sampletime = round(1/50,3) #
#             profilo, xscale  = Profi.Acquire_profilo(xScale = 1, ProfilesPerFrame = 1)
#             SaveProfilo(profilo, xscale, include_dir)
#     elif msg.data == False:
#         print('sono disconnesso')
#     rospy.spin()
