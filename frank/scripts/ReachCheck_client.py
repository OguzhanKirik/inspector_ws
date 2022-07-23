#!/usr/bin/env python3
import rospy
import numpy as np
from frank.msg import baseStruct, structArray
from frank.srv import *

TocheckStruct=[]
StructChecked=[]

TocheckStruct=structArray()
StructChecked=structArray()
TocheckStruct.Procedure=0
if TocheckStruct.Procedure==0:
    Targ_fori=np.array(np.loadtxt(str(rospy.get_param("/PosePath/drill"))+"/profi_tcp_pose1.txt", usecols=range(8)))
else:
    Targ_fori=np.array(np.loadtxt(str(rospy.get_param("/PosePath/drill"))+"/drill_tcp_pose1.txt", usecols=range(8)))

Ordered_tag_fori=np.ones((len(Targ_fori),9))*(-2)
Ordered_tag_fori[:,0:8]=Targ_fori
print(Ordered_tag_fori)



#Create the right structure
for i in range(len(Ordered_tag_fori)):
    Tocheck=[]
    Tocheck=baseStruct()
    Tocheck.ID=i
    Tocheck.drillOp=3
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
        GenPath.PlanOut.FilePath= "/home/c301/Desktop/TEMP/DENSE"
        finalreq=ask_state(GenPath.PlanOut)
        
    else:
        TocheckStruct.FilePath = "/home/c301/Desktop/TEMP/SPARSE"
        finalreq=ask_state(TocheckStruct)
except rospy.ServiceException as e:
    print("Service call failed: %s"%e)    