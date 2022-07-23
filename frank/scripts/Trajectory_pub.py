#!/usr/bin/env python3
#from frank.scripts.Joint_traj import L6_targ
from ast import While
from operator import mul

import rospy
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, Transform
from std_msgs.msg import Int16
from std_msgs.msg import Bool
from std_msgs.msg import Header

from frank.srv import *
from Services import *
from frank.msg import OperationState

import numpy as np
import math

from scipy.spatial.distance import pdist, squareform
import scipy as sp
from scipy.optimize import curve_fit
from Util import*
from irb4600IK import*
import time as tm
State_trigg=-100
Path_fori=None
Idle_flag=True
PathReady=False

def callback(data):
    rospy.loginfo(data.data)

def handle_StateReq(request):
    global State_trigg
    global Path_fori
    global PathReady
    State_trigg=request.Path.Procedure
    if State_trigg ==1:
        DrillOp=request.Path.structList[0].drillOp
        rospy.set_param("/Procedure/DrillOp", DrillOp)
    elif State_trigg == 0:
        PathDenseScan= str(request.Path.FilePath)
        rospy.set_param("/FilePath/dense", PathDenseScan)
    elif State_trigg == 2:
        PathSparseScan= str(request.Path.FilePath)
        rospy.set_param("/FilePath/sparse", PathSparseScan)
          
    elif State_trigg==-4:
        rospy.set_param("/FilePath/sparse", str("/home/oguz/Desktop/TEMP/CALIB_3DCAM"))
    if State_trigg==1:
        iterations=len(request.Path.structList)
        Path_fori=np.zeros((iterations,8))
        resp=StateReqResponse()
        for i in range(iterations):
            Path_fori[i,0]=request.Path.structList[i].label
            Path_fori[i,1]=request.Path.structList[i].Pose.position.x
            Path_fori[i,2]=request.Path.structList[i].Pose.position.y
            Path_fori[i,3]=request.Path.structList[i].Pose.position.z
            Path_fori[i,4]=request.Path.structList[i].Pose.orientation.w
            Path_fori[i,5]=request.Path.structList[i].Pose.orientation.x
            Path_fori[i,6]=request.Path.structList[i].Pose.orientation.y
            Path_fori[i,7]=request.Path.structList[i].Pose.orientation.z
    if State_trigg==0 or State_trigg==-5:
     
        iterations=len(request.Path.structList)
        Path_fori=np.zeros((iterations,8))
        resp=StateReqResponse()
        for i in range(iterations):
            Path_fori[i,0]=request.Path.structList[i].label
            Path_fori[i,1]=request.Path.structList[i].Pose.position.x
            Path_fori[i,2]=request.Path.structList[i].Pose.position.y
            Path_fori[i,3]=request.Path.structList[i].Pose.position.z
            Path_fori[i,4]=request.Path.structList[i].Pose.orientation.w
            Path_fori[i,5]=request.Path.structList[i].Pose.orientation.x
            Path_fori[i,6]=request.Path.structList[i].Pose.orientation.y
            Path_fori[i,7]=request.Path.structList[i].Pose.orientation.z
        # # # #####
        # # # ## temporary substitute with 3d camera dense scaN
        # # # #####
        # # # ###############################################################################################################
        # rospy.wait_for_service('connect_3dcamerax36',timeout=1)
        # print('connecting to 3d camera x36')
        # conn3d = rospy.ServiceProxy("connect_3dcamerax36",Connect3DCamerax36)
        # resp = conn3d()
        # State_pub=OperationState()
        # State_pub.procedure=State_trigg
        # State_pub.state="Started"
        # State_pub.error="No error"
        # StatePub.publish(State_pub)

        # for i in range(len(Path_fori)):
        #     num = Path_fori[i,0]
        #     # if not os.path.isdir(filepath+"/"+str(int(num_new))):
        #     # os.mkdir(filepath+"/"+str(int(num_new)))
        #     # filepath1 = filepath+"/"+str(int(num_new))
        #     # rospy.set_param("FilePath/sparse", filepath1)
        #             #go to position
        #     s_gotopos = rospy.ServiceProxy("GoToPose", GoToPose)
        #     req = GoToPoseRequest()
        #     target_point = Path_fori[i,1:8]
        #     target_point = [float(x) for x in target_point]
        #     print(target_point)
        #     req.pose = list(target_point)
        #     response = s_gotopos(req)
        #             #grab 3d photo
            
        #     rospy.wait_for_service('grab_3dpointcloudx36',timeout=1)
        #     print('grab 3d snapshot from 3d camera')
            
        #     grab3d = rospy.ServiceProxy("grab_3dpointcloudx36",Grab3DPointcloudx36)
        #     req2 = Grab3DPointcloudx36Request()
        #     req2.num = int(num)
        #     resp = grab3d(req2)

        #     #statepub = rospy.Publisher("/operation_state", OperationState, queue_size =1)
            
        #     msg = OperationState()
        #     msg.procedure = 0
        #     msg.state = "cluster saved :  " + str(int(num))
        #     msg.error = "No error"
        #     StatePub.publish(msg)

        #     print(" I sent the message of the cluster num :  "  + str(num))
        # tm.sleep(1)
        # #goHome
        # pubnum.publish(-1)
        # tm.sleep(1)
        # State_pub=OperationState()
        # State_pub.procedure=State_trigg
        # State_pub.state="Completed"
        # State_pub.error="No error"
        # StatePub.publish(State_pub)

        # # # #######################################################################################################


##########
        #############
        
        print("____ADDING THE PREPATH_______")
        PrePath=np.loadtxt("/home/oguz/Desktop/TEMP/DRILL/PrePath.txt")
        iterations=len(request.Path.structList)
        Path_fori=np.zeros((iterations+len(PrePath),8))
        resp=StateReqResponse()
        for i in range(len(PrePath)):
            Path_fori[i,0]=int(PrePath[i,0])
            Path_fori[i,1]=PrePath[i,1]
            Path_fori[i,2]=PrePath[i,2]
            Path_fori[i,3]=PrePath[i,3]
            Path_fori[i,4]=PrePath[i,4]
            Path_fori[i,5]=PrePath[i,5]
            Path_fori[i,6]=PrePath[i,6]
            Path_fori[i,7]=PrePath[i,7]
        for i in range(iterations):
            Path_fori[i+len(PrePath),0]=request.Path.structList[i].label
            Path_fori[i+len(PrePath),1]=request.Path.structList[i].Pose.position.x
            Path_fori[i+len(PrePath),2]=request.Path.structList[i].Pose.position.y
            Path_fori[i+len(PrePath),3]=request.Path.structList[i].Pose.position.z
            Path_fori[i+len(PrePath),4]=request.Path.structList[i].Pose.orientation.w
            Path_fori[i+len(PrePath),5]=request.Path.structList[i].Pose.orientation.x
            Path_fori[i+len(PrePath),6]=request.Path.structList[i].Pose.orientation.y
            Path_fori[i+len(PrePath),7]=request.Path.structList[i].Pose.orientation.z

    if State_trigg==-5:
        rospy.set_param("/FilePath/dense", str("/home/oguz/Desktop/TEMP/CALIB_PROFI"))
        print("ho settato il filepath : "+ str(rospy.get_param("/FilePath/dense")))
        #Path_fori=np.array(np.loadtxt('/home/c301/Desktop/ROS-Frank_robot_control/catkin_ws/src/frank/Data/Path_Calib_profi.txt', usecols=range(8)))  
  
        
    resp=str("State request received")
    PathReady=True
    return StateReqResponse(resp)




rospy.init_node("Trajectory_pub")

rospy.Service('reach_check', ReachCheck, handle_ReachCheck)
rospy.Service('path_plan', PathPlan, handle_PathPlan)
rospy.Service('state_request',StateReq,handle_StateReq)
###Sensor services
connprof = rospy.ServiceProxy('connect_profilometer',ConnectProfilometer)
disconnprof = rospy.ServiceProxy('disconnect_profilometer',DisconnectProfilometer)
conn3d = rospy.ServiceProxy("connect_3dcamera",Connect3DCamera)
disconn3d = rospy.ServiceProxy("disconnect_3dcamera",Disconnect3DCamera)
startProfi=rospy.ServiceProxy('start_profi_acquisition',StartProfi)
stopProfi=rospy.ServiceProxy('stop_profi_acquisition',StopProfi)
starsavetProfi=rospy.ServiceProxy('request_save_profi',RequestSaveProfi)
stopsaveProfi=rospy.ServiceProxy('request_stop_save_profi',RequestStopSaveProfi)
startDrill=rospy.ServiceProxy('plc_command',CommandPLC)
DrillSpeed=rospy.ServiceProxy("plc_read", ReadPLC)
###CAD alignment services
#alignSparse=rospy.ServiceProxy('sparse_alignment', SparseAlignment)
#alignDense=rospy.ServiceProxy('dense_alignment', DenseAlignment)

StatePub=rospy.Publisher("/operation_state", OperationState, queue_size=1)

pub = rospy.Publisher("/robot_set_point", MultiDOFJointTrajectoryPoint, queue_size=0)
jpub = rospy.Publisher("/robot_jset_point", JointState, queue_size=0)

pubnum = rospy.Publisher("/num_to_check", Int16, queue_size=1)



    

##########################Read file, Cluster, Find extremes of clusters######################################################

#################################################
try:
    while True:  #State Machine  
        
        if State_trigg==1 or State_trigg==-5 or State_trigg==0: # or  #0:Scan   #1:Drilling  #-5 Prof Cal
            #inform about the start of procedure
            State_pub=OperationState()
            State_pub.procedure=State_trigg
            State_pub.state="Started"
            State_pub.error="No error"
            StatePub.publish(State_pub)
            Drill=State_trigg
            rospy.set_param("/Procedure/case", Drill)
            pubnum.publish(-2)
            # set the correct tool
            print("ASk for tool change")

                
            ####Pramaters for main procedures
            Vsafe=rospy.get_param("/DrillingParam/SpeedInSafe")
            
            if Drill==1:
                omega=math.radians(rospy.get_param("/DrillingParam/OmegaDot"))
                HoleAppr=rospy.get_param("/DrillingParam/HoleAppr")
                HoleInit=rospy.get_param("/DrillingParam/HoleInit")
                Godeep=rospy.get_param("/DrillingParam/HoleDepth")
                Vc=rospy.get_param("/DrillingParam/MoveSpeed")
               
                Vdrill=rospy.get_param("/DrillingParam/FeedSpeed")
                Min_dist=rospy.get_param("/DrillingParam/MinDist") # 472 max distance tool-J5 + 135 length of J5
                N_dis=rospy.get_param("/DrillingParam/PossRot")
                rospy.sleep(0.5) #give rws time to update tool on irc5
                
            else:
                omega=math.radians(rospy.get_param("/DenseScanParam/OmegaDot"))
                Vscan=rospy.get_param("/DenseScanParam/ScanSpeed")
                GoProfi=True
                GosaveP=True
                Vc=rospy.get_param("/DenseScanParam/MoveSpeed")
                HoleAppr=0
                Min_dist=rospy.get_param("/DenseScanParam/MinDist")
                N_dis=rospy.get_param("/DenseScanParam/PossRot")
                
                connprof()
         
            
            dTheta=2*math.pi/N_dis
            #################################################
            #################################################
            #Build Safe Surface for Joints 1-2-3 motion ("Obstacle Avoidance")
    
           
            popt0=rospy.get_param('/DrillingParam/popt0')
            popt1=rospy.get_param('/DrillingParam/popt1')
            popt2=rospy.get_param('/DrillingParam/popt2')
            popt3=rospy.get_param('/DrillingParam/popt3')
            popt4=rospy.get_param('/DrillingParam/popt4')
            popt=[popt0, popt1, popt2, popt3, popt4]  

            #Utilities
            rate=rospy.Rate(250)
            egm_time_step=0.004
            time_step=0.004
            mmtom=0.001
        
            ###Joint limits####
            MIN_angle=[math.radians(-180),math.radians(-90),math.radians(-180),math.radians(-400),math.radians(-125),math.radians(-400)]
            MAX_angle=[math.radians(180),math.radians(150),math.radians(75),math.radians(400),math.radians(120),math.radians(400)]
            
            #MIN_angle=[math.radians(-170),math.radians(-65),math.radians(-180),math.radians(-300),math.radians(-130),math.radians(-360)]
            #MAX_angle=[math.radians(170),math.radians(85),math.radians(70),math.radians(300),math.radians(130),math.radians(360)]
            ###Here add take data from Param service
            if Drill==1:
               DrillOp=rospy.get_param("/Procedure/DrillOp")
               if DrillOp==1 or DrillOp==4:
                   rospy.set_param("DrillingParam/HoleDepth",3)
               elif DrillOp==2 or DrillOp==5:
                   rospy.set_param("DrillingParam/HoleDepth",10)
               elif DrillOp==3:
                   rospy.set_param("DrillingParam/HoleDepth",1)        
               Godeep=rospy.get_param("DrillingParam/HoleDepth")
               DrillOp=str(DrillOp)
               toolx=rospy.get_param("/DTool_ref"+DrillOp+"/x")
               tooly=rospy.get_param("/DTool_ref"+DrillOp+"/y")
               toolz=rospy.get_param("/DTool_ref"+DrillOp+"/z")
               toolq1=rospy.get_param("/DTool_ref"+DrillOp+"/q1")
               toolq2=rospy.get_param("/DTool_ref"+DrillOp+"/q2")
               toolq3=rospy.get_param("/DTool_ref"+DrillOp+"/q3")
               toolq4=rospy.get_param("/DTool_ref"+DrillOp+"/q4")
               ########Drill Bit Manager#######
               if rospy.get_param("/DrillBit/actual") == 0: #No drillbit
                    target=rospy.get_param("/DrillBit/"+DrillOp+"_pos") #position of target drillbit
                    rospy.set_param("/DrillBit/target", target)
                    pubnum.publish(8) #loadDrillbit
                    while (rospy.get_param("/DrillBit/actual") == 0):
                        pass
               elif rospy.get_param("/DrillBit/actual")== int(DrillOp):
                    pass
               else:
                    target=rospy.get_param("/DrillBit/"+DrillOp+"_pos")
                    rospy.set_param("/DrillBit/target", target)
                    ###to finish
                    pubnum.publish(9) #loadDrillbit
                    while (rospy.get_param("/DrillBit/actual") != int(DrillOp)):
                        pass
            else:
               toolx=rospy.get_param("/STool_ref/x")
               tooly=rospy.get_param("/STool_ref/y")
               toolz=rospy.get_param("/STool_ref/z")
               toolq1=rospy.get_param("/STool_ref/q1")
               toolq2=rospy.get_param("/STool_ref/q2")
               toolq3=rospy.get_param("/STool_ref/q3")
               toolq4=rospy.get_param("/STool_ref/q4")
            
            woobjx=rospy.get_param("/Wobj_ref/ufx")
            woobjy=rospy.get_param("/Wobj_ref/ufy")
            woobjz=rospy.get_param("/Wobj_ref/ufz")
            
            
            tool=[toolx,tooly,toolz,toolq1,toolq2,toolq3,toolq4]
            obj=[woobjx,woobjy,woobjz] 
            
            #DH_param
            a1=rospy.get_param("/DH_par/a1")
            a2=rospy.get_param("/DH_par/a2")
            a3=rospy.get_param("/DH_par/a3")
            a4=rospy.get_param("/DH_par/a4")
            a5=rospy.get_param("/DH_par/a5")
            a6=rospy.get_param("/DH_par/a6")
            dh_a=[a1,a2,a3,a4,a5,a6]
            d1=rospy.get_param("/DH_par/d1")
            d2=rospy.get_param("/DH_par/d2")
            d3=rospy.get_param("/DH_par/d3")
            d4=rospy.get_param("/DH_par/d4")
            d5=rospy.get_param("/DH_par/d5")
            d6=rospy.get_param("/DH_par/d6")
            dh_d=[d1,d2,d3,d4,d5,d6]
            alpha1=rospy.get_param("/DH_par/alpha1")
            alpha2=rospy.get_param("/DH_par/alpha2")
            alpha3=rospy.get_param("/DH_par/alpha3")
            alpha4=rospy.get_param("/DH_par/alpha4")
            alpha5=rospy.get_param("/DH_par/alpha5")
            alpha6=rospy.get_param("/DH_par/alpha6")
            dh_alpha=[alpha1,alpha2,alpha3,alpha4,alpha5,alpha6]
            
            ActiveP = False
            
          
           
            
            ####create goal structure##########
            goal = MultiDOFJointTrajectoryPoint()
            goal.transforms.append(Transform())
            goal.velocities.append(Twist())
            
            robot_setpoints = MultiDOFJointTrajectoryPoint()
            robot_setpoints.transforms.append(Transform())
            robot_setpoints.velocities.append(Twist())
            
            j_robot_setpoints = JointState()
            ####get robot pose#################
            robot_feedback=MultiDOFJointTrajectoryPoint()
            j_robot_feedback=JointState()
           
            ####################################################Generate Drilling Path################################################################################################################
            while PathReady==False: #Sync with service
                rospy.sleep(0.01)
            PathReady=False    
            
            if (Drill==1):
                lpf=len(Path_fori)
                l_tpf=lpf+4*lpf
                Temp_Path_fori=np.zeros((l_tpf,8))
                for q in range(lpf):
            
            
                    Temp_Path_fori[q*5,:]=AlongZ(Path_fori[q,:], HoleAppr)  ###Approach drill hole
                    Temp_Path_fori[q*5+1,:]=AlongZ(Path_fori[q,:], HoleInit) ###Go close to the hole
                    Temp_Path_fori[q*5+2,:]=AlongZ(Path_fori[q,:],Godeep)  ###Go deep
                    Temp_Path_fori[q*5+3,:]=AlongZ(Path_fori[q,:], HoleInit) ##Go Out slowly
                    Temp_Path_fori[q*5+4,:]=Temp_Path_fori[q*5,:]      ###Go out Faster
            
                Path_fori=Temp_Path_fori
            print("Final Path fori")
            print(Path_fori)
            
            counter=0

            
            x_arr=[]
            y_arr=[]
            z_arr=[]
            t_arr=[]
            ###Test try to interpolate angles
            qx_arr=[]
            qy_arr=[]
            qz_arr=[]
            qw_arr=[]
            
            traj_length=0
            traj_len_arr=[]
            sol_fin=[]
            
            for i in range(len(Path_fori)-1):
                
                if (i==0 or Path_fori[i,0]!=Path_fori[i-1,0]):
                    tcp_fin=[Path_fori[i,1],Path_fori[i,2],Path_fori[i,3],Path_fori[i,4],Path_fori[i,5],Path_fori[i,6],Path_fori[i,7]]
                    L6_fin=FromTCPtoLink6(tcp_fin,tool)
                    L6_fin[0]=(L6_fin[0]+obj[0])*mmtom
                    L6_fin[1]=(L6_fin[1]+obj[1])*mmtom
                    L6_fin[2]=(L6_fin[2]+obj[2])*mmtom
    
                    sol_fin.append(IK_irb4600_Red(dh_d,dh_a,dh_alpha,L6_fin,0,Drill))
            #print(sol_fin)
            
            ####Drilling trajectories generation###################
            if Drill==1:    
                
                mul_of5_1 = 2 ###For drilling
                mul_of5_2 = mul_of5_1+1
                
                for i in range(1,len(Path_fori)):#range(length):  
                    
                    goalx= Path_fori[i,1]
                    goaly= Path_fori[i,2]
                    goalz= Path_fori[i,3]
                   
                    goalqx= Path_fori[i,5]
                    goalqy= Path_fori[i,6]
                    goalqz= Path_fori[i,7]
                    goalqw= Path_fori[i,4] #this is the label of traj
                    ####Clear array####
                    qx_arr0=[]
                    qy_arr0=[]
                    qz_arr0=[]
                    qw_arr0=[]
    
                    if (i==mul_of5_1): ####Frank is Drilling!!
                        Des_vel=Vdrill
                        traj_time = Const_vel(Path_fori[i-1,1],Path_fori[i-1,2],Path_fori[i-1,3],goalx,goaly,goalz, Des_vel)
                          ##Create interpolation set points####
                        x_arr.append(interpolate(Path_fori[i-1,1], goalx, traj_time,egm_time_step))
                        y_arr.append(interpolate(Path_fori[i-1,2], goaly, traj_time,egm_time_step))
                        z_arr.append(interpolate(Path_fori[i-1,3], goalz, traj_time,egm_time_step))
                        n_steps=int((traj_time)/egm_time_step)
                        if n_steps<50:
                            n_steps=50
                        T_arr=np.linspace(0,traj_time,n_steps)
                        
                        qw_arr0, qx_arr0, qy_arr0, qz_arr0 = slerp(Path_fori[i-1,4],Path_fori[i-1,5],Path_fori[i-1,6],Path_fori[i-1,7], goalqw, goalqx, goalqy, goalqz, traj_time, T_arr)

                        qw_arr.append(qw_arr0)
                        qx_arr.append(qx_arr0)
                        qy_arr.append(qy_arr0)
                        qz_arr.append(qz_arr0)


                        
                        mul_of5_1=mul_of5_1+5
                    
                    elif (i==mul_of5_2):
                        Des_vel=Vdrill
                        traj_time = Const_vel(Path_fori[i-1,1],Path_fori[i-1,2],Path_fori[i-1,3],goalx,goaly,goalz, Des_vel)
                          ##Create interpolation set points####
                        x_arr.append(interpolate(Path_fori[i-1,1], goalx, traj_time,egm_time_step))
                        y_arr.append(interpolate(Path_fori[i-1,2], goaly, traj_time,egm_time_step))
                        z_arr.append(interpolate(Path_fori[i-1,3], goalz, traj_time,egm_time_step))
                        n_steps=int((traj_time)/egm_time_step)
                        if n_steps<50:
                            n_steps=50
                        T_arr=np.linspace(0,traj_time,n_steps)
                        
                        qw_arr0, qx_arr0, qy_arr0, qz_arr0 = slerp(Path_fori[i-1,4],Path_fori[i-1,5],Path_fori[i-1,6],Path_fori[i-1,7], goalqw, goalqx, goalqy, goalqz, traj_time, T_arr)

                        
                        qw_arr.append(qw_arr0)
                        qx_arr.append(qx_arr0)
                        qy_arr.append(qy_arr0)
                        qz_arr.append(qz_arr0)
                        
                        mul_of5_2=mul_of5_2+5
                    
                    else:
                        Des_vel=Vc#*3#*4 # Res*f -> 0,2 mm * 250Hz
                        traj_time = Const_vel(Path_fori[i-1,1],Path_fori[i-1,2],Path_fori[i-1,3],goalx,goaly,goalz, Des_vel)
                          ##Create interpolation set points####
                        x_arr.append(interpolate(Path_fori[i-1,1], goalx, traj_time,egm_time_step))
                        y_arr.append(interpolate(Path_fori[i-1,2], goaly, traj_time,egm_time_step))
                        z_arr.append(interpolate(Path_fori[i-1,3], goalz, traj_time,egm_time_step))
                        n_steps=int((traj_time)/egm_time_step)
                        if n_steps<50:
                            n_steps=50
                        T_arr=np.linspace(0,traj_time,n_steps)
                        
                        qw_arr0, qx_arr0, qy_arr0, qz_arr0 = slerp(Path_fori[i-1,4],Path_fori[i-1,5],Path_fori[i-1,6],Path_fori[i-1,7], goalqw, goalqx, goalqy, goalqz, traj_time, T_arr)
        
                        qw_arr.append(qw_arr0)
                        qx_arr.append(qx_arr0)
                        qy_arr.append(qy_arr0)
                        qz_arr.append(qz_arr0)
                
                DrillingOperation=rospy.get_param("/Procedure/DrillOp")
                DrillComm=CommandPLCRequest()
                DrillComm.command="GVL_ROS.EM_SetpointSpeed"
                if DrillingOperation == 1 or DrillingOperation==4:    
                    DrillComm.value=72000 #12000 rpm                                 
                elif DrillingOperation == 2 or DrillingOperation==5:    
                    DrillComm.value=36000 #6000 rpm              
                else:
                    DrillComm.value=0 #0 rpm
                startDrill(DrillComm)

                rospy.sleep(1)

                Dspeed=ReadPLCRequest()
                Dspeed.command="GVL_ROS.EM_RotSpeed"
                ActualVel=ReadPLCResponse()
                ActualVel=DrillSpeed(Dspeed)
                while (DrillComm.value>0 and ActualVel.value<20000):
                    Dspeed=ReadPLCRequest()
                    Dspeed.command="GVL_ROS.EM_RotSpeed"
                    ActualVel=ReadPLCResponse()
                    ActualVel=DrillSpeed(Dspeed)
                    rospy.sleep(1)




            ####DENSEScan trajectories generation###################       
            number=0
            if Drill==0 or Drill==-5:
                joints_car=np.empty((6,0))
                for i in range(1,len(Path_fori)):
                    goalx= Path_fori[i,1]
                    goaly= Path_fori[i,2]
                    goalz= Path_fori[i,3]
                    goalqx= Path_fori[i,5]
                    goalqy= Path_fori[i,6]
                    goalqz= Path_fori[i,7]
                    goalqw= Path_fori[i,4] #this is the label of traj
                    ####Clear array####
                    qx_arr0=[]
                    qy_arr0=[]
                    qz_arr0=[]
                    qw_arr0=[]
                    Des_vel=Vscan # Res*f -> 0,2 mm * 250Hz
                    traj_time = Const_vel(Path_fori[i-1,1],Path_fori[i-1,2],Path_fori[i-1,3],goalx,goaly,goalz, Des_vel)
                      ##  ##Create interpolation set points####
                    x_arr.append(interpolate(Path_fori[i-1,1], goalx, traj_time,egm_time_step))
                    y_arr.append(interpolate(Path_fori[i-1,2], goaly, traj_time,egm_time_step))
                    z_arr.append(interpolate(Path_fori[i-1,3], goalz, traj_time,egm_time_step))
                    n_steps=int((traj_time)/egm_time_step)
                    if n_steps<50:
                            n_steps=50
                    T_arr=np.linspace(0,traj_time,n_steps)
                    qw_arr0, qx_arr0, qy_arr0, qz_arr0 = slerp(Path_fori[i-1,4],Path_fori[i-1,5],Path_fori[i-1,6],Path_fori[i-1,7], goalqw, goalqx, goalqy, goalqz, traj_time, T_arr)
                    qw_arr.append(qw_arr0)
                    qx_arr.append(qx_arr0)
                    qy_arr.append(qy_arr0)
                    qz_arr.append(qz_arr0)
                    if (Path_fori[i,0]==Path_fori[i-1,0]):
                        for t in range(len(x_arr[i-1])):
                            tcp_int=[]
                            tcp_int=[x_arr[i-1][t],y_arr[i-1][t],z_arr[i-1][t],qw_arr[i-1][t],qx_arr[i-1][t],qy_arr[i-1][t],qz_arr[i-1][t]]
                            #tcp_int=[x_arr[t],y_arr[t],z_arr[t],goal.transforms[0].rotation.w,goal.transforms[0].rotation.x,goal.transforms[0].rotation.y,goal.transforms[0].rotation.z]
                            L6_car=FromTCPtoLink6(tcp_int,tool)
                            L6_car[0]=(L6_car[0]+obj[0])*mmtom
                            L6_car[1]=(L6_car[1]+obj[1])*mmtom
                            L6_car[2]=(L6_car[2]+obj[2])*mmtom
                            #print(L6_car)
                            sol_car=IK_irb4600_Red(dh_d,dh_a,dh_alpha,L6_car,0,Drill)
                            #print("sol_car"+str(np.degrees(sol_car)))
                            #joints_car_temp=choose_min_joints(sol_car,min_angle,max_angle)
                            if number==0:
                                joints_car_temp=choose_min_joints(sol_car,min_angle,max_angle)
                                joints_car_temp=np.squeeze(joints_car_temp)
                            else:
                                actual_joints=joints_car[:,number-1]
                                joints_car_temp=choose_next_joints(actual_joints, sol_car,MIN_angle,MAX_angle,2)
                                joints_car_temp=np.squeeze(joints_car_temp)
                            joints_car=np.column_stack((joints_car,joints_car_temp))
                            number=number+1
    
            
            ind=0
            indj=0
            
            for i in range(len(Path_fori)):#range(length):  
    
                #print("ROBOT_GOAL is:" + str(goal.transforms[0]))   
                print("ACTIVE IS: "+str(ActiveP))
                print(Path_fori[i,0])
                
                ####Create two different cases : 1) i'm performing the given trajectory (reduced speed) 2) i'm moving to another trajectory (max speed)
                if (i>0 and Path_fori[i,0]==Path_fori[i-1,0]):
                    t0=rospy.Time.now()
                    time0=t0.secs+t0.nsecs*1e-9
                    if  Drill==1 and ActiveP==True:
                       print("request of egm pose mode")
                       pubnum.publish(1)
                       rospy.sleep(1)
                    if Drill==0 or Drill==-5:
                       if GoProfi==True:
                            req = StartProfiRequest()
                            req.label = int(Path_fori[i,0])
                            res = startProfi(req)
                            print("____ricevuto risposta Profi partito___" + res.error)
                            GoProfi=False
                       if GosaveP==True:
                            sreq = RequestSaveProfiRequest()
                            sreq.label =  int(Path_fori[i,0])
                            res = starsavetProfi(sreq)
                            GosaveP=False

                       rospy.set_param("/EGM/mod", 3) #give both joint and pose feedback 
            
                       
                    J=0
                    for j in range(len(x_arr[i-1])):
                       tint=rospy.Time.now()
                       timeint=tint.secs+tint.nsecs*1e-9
                       
                       
                       if j==0 and ActiveP==True and Drill==1:
                           for t in range(100):
                               robot_setpoints.transforms[0].translation.x = x_arr[i-1][0]
                               robot_setpoints.transforms[0].translation.y = y_arr[i-1][0]
                               robot_setpoints.transforms[0].translation.z = z_arr[i-1][0]
                               
                               robot_setpoints.transforms[0].rotation.x = qx_arr[i-1][0]
                               robot_setpoints.transforms[0].rotation.y = qy_arr[i-1][0]
                               robot_setpoints.transforms[0].rotation.z = qz_arr[i-1][0]
                               robot_setpoints.transforms[0].rotation.w = qw_arr[i-1][0]
                               pub.publish(robot_setpoints)
                       
                               rospy.sleep(egm_time_step)
                           ActiveP=False     
            
                       robot_setpoints.transforms[0].translation.x = x_arr[i-1][j]
                       robot_setpoints.transforms[0].translation.y = y_arr[i-1][j]
                       robot_setpoints.transforms[0].translation.z = z_arr[i-1][j]
                       
                       robot_setpoints.transforms[0].rotation.x = qx_arr[i-1][j]
                       robot_setpoints.transforms[0].rotation.y = qy_arr[i-1][j]
                       robot_setpoints.transforms[0].rotation.z = qz_arr[i-1][j]
                       robot_setpoints.transforms[0].rotation.w = qw_arr[i-1][j]
                       t=rospy.Time.now()
                       time=t.secs+t.nsecs*1e-9-(t0.secs+t0.nsecs*1e-9)
                       
                       robot_setpoints.time_from_start=rospy.Duration(time)
                       if Drill==0 or Drill==-5:
                        robot_setpoint_joints[0] = math.degrees(joints_car[0,ind+j])
                        robot_setpoint_joints[1] = math.degrees(joints_car[1,ind+j])
                        robot_setpoint_joints[2] = math.degrees(joints_car[2,ind+j])
                        robot_setpoint_joints[3] = math.degrees(joints_car[3,ind+j])
                        robot_setpoint_joints[4] = math.degrees(joints_car[4,ind+j])
                        robot_setpoint_joints[5] = math.degrees(joints_car[5,ind+j])
                        j_robot_setpoints.position=tuple(robot_setpoint_joints)
                        jpub.publish(j_robot_setpoints)
                        
                        #ind=ind+1
                       else:  
                       #print("X is:" + str(x_arr))
                        pub.publish(robot_setpoints)
                       #jpub.publish(j_robot_setpoints)
                       t=rospy.Time.now()
                       time=t.secs+t.nsecs*1e-9
                       if j==0 :
                           fix_time_step=egm_time_step-(time-time0)
                       else:
                           fix_time_step=egm_time_step-(time-timeint)
                       if fix_time_step<0:
                            fix_time_step=0
                       #print(fix_time_step)
                       rospy.sleep(fix_time_step)
                       J=j+1   
                    #pubnum.publish(0)
                    ind=ind+J
                else :
                    
                    ###Move J####
                    if Drill==1 or i==0:
                        pubnum.publish(2)
                        print("request of egm joint mode")
                        rospy.sleep(1) 
                    if Drill==0 or Drill==-5:
                        GosaveP=True
                        if i>0:
                            stopsaveProfi()
                        
                        rospy.set_param("/EGM/mod",1)       
                    
                    ActiveP=True
                    
                    t0j=rospy.Time.now()
                    time0j=t0j.secs+t0j.nsecs*1e-9
                    
                    interp_final=[]
                    joints_fin=[]
                    
                    j_robot_feedback=rospy.wait_for_message("/joints_robot_feedback", JointState) 
                   
                    actual_joints=np.array([[j_robot_feedback.position[0]],[j_robot_feedback.position[1]],[j_robot_feedback.position[2]],[j_robot_feedback.position[3]],[j_robot_feedback.position[4]],[j_robot_feedback.position[5]]])
                    print(actual_joints)
     
                    if Drill==1 :
                       
                        joints_fin=choose_min_joints(sol_fin[indj],MIN_angle,MAX_angle)
                        
                        interp_final=Move_safely(np.squeeze(actual_joints),np.squeeze(joints_fin),popt, Vsafe,omega,egm_time_step,tool,obj)
                    else:
                        interp_final=j_int(actual_joints,joints_car[:,ind],omega,egm_time_step)

                    indj=indj+1
                    j_robot_setpoints = j_robot_feedback
                    robot_setpoint_joints=list(j_robot_setpoints.position)

                    
                    for ii in range(interp_final.shape[1]+200):#+200
                       tintj=rospy.Time.now()
                       timeintj=tintj.secs+tintj.nsecs*1e-9
                       #robot_setpoints.transforms[0].translation.x = interp_final.shape[1]-ii-1
                       #pubCheck.publish(robot_setpoints)
                       if ii < interp_final.shape[1]:
                            robot_setpoint_joints[0] = math.degrees(interp_final[0][ii])
                            robot_setpoint_joints[1] = math.degrees(interp_final[1][ii])
                            robot_setpoint_joints[2] = math.degrees(interp_final[2][ii])
                            robot_setpoint_joints[3] = math.degrees(interp_final[3][ii])
                            robot_setpoint_joints[4] = math.degrees(interp_final[4][ii])
                            robot_setpoint_joints[5] = math.degrees(interp_final[5][ii])
                       else: ##To avoid lag time due to udp protocol
                            robot_setpoint_joints[0] = math.degrees(interp_final[0][-1])
                            robot_setpoint_joints[1] = math.degrees(interp_final[1][-1])
                            robot_setpoint_joints[2] = math.degrees(interp_final[2][-1])
                            robot_setpoint_joints[3] = math.degrees(interp_final[3][-1])
                            robot_setpoint_joints[4] = math.degrees(interp_final[4][-1])
                            robot_setpoint_joints[5] = math.degrees(interp_final[5][-1])
            
                         
            
                    
                       j_robot_setpoints.position=tuple(robot_setpoint_joints)
                       
                       
                       jpub.publish(j_robot_setpoints)
                       tj=rospy.Time.now()
                       timej=tj.secs+tj.nsecs*1e-9
                       if ii==0 :
                        fix_time_stepj=egm_time_step-(timej-time0j)
                       else:
                          fix_time_stepj=egm_time_step-(timej-timeintj)
                       if fix_time_stepj<0:
                            fix_time_stepj=0
                       rospy.sleep(egm_time_step)
            stopsaveProfi()
            rospy.sleep(1.1)
            pubnum.publish(-1)
            GoProfi=True
            GosaveP=True
            stopProfi()
            Idle_flag=True
            Path_fori=[]
            
    
            State_pub=OperationState()
            State_pub.procedure=State_trigg
            State_pub.state="Completed"
            State_pub.error="No error"
            StatePub.publish(State_pub)

            DrillComm=CommandPLCRequest()
            DrillComm.command="GVL_ROS.EM_SetpointSpeed"
            DrillComm.value=0               
            startDrill(DrillComm)
            #if State_trigg==0:
            #    rospy.sleep(5)
            #    alignDense()
            State_trigg=-100
        
        elif State_trigg==2 or State_trigg==3 or State_trigg==4 or State_trigg==5 or State_trigg==6 or State_trigg==-4 or State_trigg==9 or State_trigg==10 or State_trigg==11 or State_trigg==12 or State_trigg==13 or State_trigg==14: #2:Sparse Scanning #3:ChangeDrill #4:Change Profi
            # rospy.wait_for_message("/rws_ready",Bool)
            pubnum.publish(0)#Enter in the rws main loop
            rospy.sleep(0.2)
            #inform about the starting of procedure
            State_pub=OperationState()
            State_pub.procedure=State_trigg
            State_pub.state="Started"
            State_pub.error="No error"
            StatePub.publish(State_pub)
            if State_trigg==2:
                conn3d()
                pubnum.publish(State_trigg+1)#THIS TRIGGER THE RAPID_ROUTINE
            elif State_trigg==3:
                pubnum.publish(State_trigg+1)
            elif State_trigg==4:
                pubnum.publish(State_trigg+1)
            elif State_trigg==5:
                pubnum.publish(State_trigg+1)
            elif State_trigg==6:
                pubnum.publish(State_trigg+1)
            elif State_trigg==9:
                pubnum.publish(State_trigg+6)
            elif State_trigg==10:
                pubnum.publish(State_trigg)
            elif State_trigg==11:
                pubnum.publish(State_trigg)
            elif State_trigg==12:
                pubnum.publish(State_trigg)
            elif State_trigg==13:
                pubnum.publish(State_trigg)
            elif State_trigg==14:
                pubnum.publish(State_trigg)
            elif State_trigg==-4:  #calib 3dcam
                conn3d()
                pubnum.publish(State_trigg)               
            #TO DO:add 3Dcam data flowing          
            rospy.set_param("/EGM/mod", 0)        
            Idle_flag=True
            State_trigg=-100    
        
        elif State_trigg==-1:# Clean Termination
            pubnum.publish(-3)
            print("Terminate state requested...")
            break
        
        else:#Idle
                
            if Idle_flag==True: ##TOFIX
                print("Idle...")
                Idle_flag=False    
            
            #continue    
    
    
    print("Clean program termination is allowed (CTRL+C)...")            
except Exception as e:
    e_string=str(e)
    State_pub.state="Error Occured!"
    State_pub.error= e_string
    StatePub.publish(State_pub)
    pubnum.publish(-3)
    print(e_string)
#     pubnum.publish(-3)
#     print("STOP ACTIVATED!! Rapid Stopped")
#     pass

           
            
        
        
                
                
        
               
          
            
    
             
         
        
    
    
    
    




    
