#!/usr/bin/env python

import rospy
from frank.msg import baseStruct, structArray
from frank.srv import PathPlan ,PathPlanResponse
from frank.srv import ReachCheck ,ReachCheckResponse

from irb4600IK import*
from Util import*

def handle_PathPlan(req):
    Proc=req.RawIn.Procedure
    iterations=len(req.RawIn.structList)
    woobjx=rospy.get_param("/Wobj_ref/ufx")
    woobjy=rospy.get_param("/Wobj_ref/ufy")
    woobjz=rospy.get_param("/Wobj_ref/ufz")
    if Proc==1:
        Starting_point=[1279-woobjx,0-woobjy, 1055-woobjz] #initial pose of robot
    else:
        Starting_point=[2023-woobjx,52-woobjy, 1039-woobjz] #initial pose of robot
    ###Create support structure
    Outstruct=np.zeros((iterations,11))
    for i in range(iterations):
        Outstruct[i,0]=req.RawIn.structList[i].label
        Outstruct[i,1]=req.RawIn.structList[i].Pose.position.x
        Outstruct[i,2]=req.RawIn.structList[i].Pose.position.y
        Outstruct[i,3]=req.RawIn.structList[i].Pose.position.z
        Outstruct[i,4]=req.RawIn.structList[i].Pose.orientation.w
        Outstruct[i,5]=req.RawIn.structList[i].Pose.orientation.x
        Outstruct[i,6]=req.RawIn.structList[i].Pose.orientation.y
        Outstruct[i,7]=req.RawIn.structList[i].Pose.orientation.z
        Outstruct[i,8]=req.RawIn.structList[i].reach
        Outstruct[i,9]=req.RawIn.structList[i].ID
        Outstruct[i,10]=req.RawIn.structList[i].drillOp
    
    labels=Outstruct[:,0]
    labels_set=set(labels)
    ##Remove no reacheble points (just in case)
    Outstruct_clear=Remove_no_reach(Outstruct, 8)
    newlabels=Outstruct_clear[:,0]
    newlabelsSet=set(newlabels)
    Extremes=Get_Extremes(Outstruct_clear[:,:9], newlabelsSet)
    len_extr=len(Extremes)
    #Now i got the extremes-> Distance matrix redesign
    Ex_dist=pdist(Extremes[:,0:3])
    Ex_Dist_mat=squareform(Ex_dist)
    for k in range(0,len(Extremes),2):
        Ex_Dist_mat[k,k+1]=0.0
        Ex_Dist_mat[k+1,k]=0.0
    Proc_Mat=Ex_Dist_mat
    for u in range(len(Extremes)):
        Proc_Mat[u,u]=np.inf #Avoid that the NN is the point itself
    Path=np.zeros((len(Extremes),1))
    start=np.zeros((len(Extremes)+1,1))#row to start (also starting point)
    start[0,0]= start_from(Starting_point, Extremes)
    step=0
    for t in range(len(Extremes)):
        NN=np.min(Proc_Mat[int(start[t,0]),:])
        Extr_NN=np.argwhere(Proc_Mat[int(start[t,0]), :]==NN)
        Proc_Mat[:,int(Extr_NN[0])]=np.inf
        Path[step,0]= Extremes[Extr_NN[0],3]
        start[step+1,0]=np.min(Extr_NN)
        step=step+1
    Path_fori=np.zeros((len(Outstruct_clear),10))
    is_label=0
    lPath=len(Path)
    delta_x=0 #delta for orientation of trajectories
    delta_y=0
    delta_z=0
    tracker=0
    for h in range(1,lPath,2):
        is_label=int(abs(round(Path[h,0]/2-0.3)))#Retrieve trajecotry associated to the extreme
        print (is_label)
        elements=[]
        elements=np.argwhere(Outstruct_clear[:,0]==is_label)
        print (elements)
        find_fin = np.argwhere(Extremes[:,3]==Path[h,0])
        find_in = np.argwhere(Extremes[:,3]==Path[h-1,0])
        delta_x= Extremes[find_fin,0]-Extremes[find_in,0]
        delta_y=Extremes[find_fin,1]-Extremes[find_in,1]
        delta_z=Extremes[find_fin,2]-Extremes[find_in,2]
        if (abs(delta_x) > abs(delta_y) and abs(delta_x) > abs(delta_z)): #If x principal components between extreme-> sort on x column
            if (Extremes[find_in,0]<Extremes[find_fin,0]):
                Path_fori[tracker:tracker+len(elements),:]=Outstruct_clear[elements[0]+np.argsort(Outstruct_clear[int(elements[0]):int(elements[-1])+1,1])]
            else:
                Path_fori[tracker:tracker+len(elements),:]=Outstruct_clear[elements[0]+np.argsort(Outstruct_clear[int(elements[0]):int(elements[-1])+1,1])[::-1]]
        elif(abs(delta_y) > abs(delta_x) and abs(delta_y) > abs(delta_z)):
            if (Extremes[find_in,1]<Extremes[find_fin,1]):
                Path_fori[tracker:tracker+len(elements),:]=Outstruct_clear[elements[0]+np.argsort(Outstruct_clear[int(elements[0]):int(elements[-1])+1,2])]
            else:
                Path_fori[tracker:tracker+len(elements),:]=Outstruct_clear[elements[0]+np.argsort(Outstruct_clear[int(elements[0]):int(elements[-1])+1,2])[::-1]]
        else:
            if (Extremes[find_in,2]<Extremes[find_fin,2]):
                Path_fori[tracker:tracker+len(elements),:]=Outstruct_clear[elements[0]+np.argsort(Outstruct_clear[int(elements[0]):int(elements[-1])+1,3])]
            else:
                Path_fori[tracker:tracker+len(elements),:]=Outstruct_clear[elements[0]+np.argsort(Outstruct_clear[int(elements[0]):int(elements[-1])+1,3])[::-1]]
        #Path_fori[tracker:tracker+len(elements),:]=Outstruct_clear[int(elements[0]):int(elements[-1])+1,:]
        tracker=tracker+len(elements)
    Path_fori=Path_fori[~np.all(Path_fori==0, axis=1)]
    Out=baseStruct()
    OutPlan=structArray()
    OutPlan.Procedure=Proc
    lPf=len(Path_fori)
    print("________ REQUEST PATH PLAN ARRIVED__________")
    print(Proc)
    # if Proc==0:
    #     print("____ADDING THE PREPATH_______")
    #     PrePath=np.loadtxt("/home/c301/Desktop/TEMP/DRILL/PrePath.txt")


    #     for i in range(len(PrePath)):
    #         Out=[]
    #         Out=baseStruct()
    #         Out.label=int(PrePath[i,0])
    #         Out.Pose.position.x=PrePath[i,1]
    #         Out.Pose.position.y=PrePath[i,2]
    #         Out.Pose.position.z=PrePath[i,3]
    #         Out.Pose.orientation.w=PrePath[i,4]
    #         Out.Pose.orientation.x=PrePath[i,5]
    #         Out.Pose.orientation.y=PrePath[i,6]
    #         Out.Pose.orientation.z=PrePath[i,7]
    #         Out.reach=int(1)
    #         OutPlan.structList.append(Out)
    for i in range(lPf):
        Out=[]
        Out=baseStruct()
        Out.ID=int(Path_fori[i,8])
        Out.drillOp=int(Path_fori[i,9])
        Out.label=int(Path_fori[i,0])
        Out.Pose.position.x=Path_fori[i,1]
        Out.Pose.position.y=Path_fori[i,2]
        Out.Pose.position.z=Path_fori[i,3]
        Out.Pose.orientation.w=Path_fori[i,4]
        Out.Pose.orientation.x=Path_fori[i,5]
        Out.Pose.orientation.y=Path_fori[i,6]
        Out.Pose.orientation.z=Path_fori[i,7]
        Out.reach=int(1)
        OutPlan.structList.append(Out)
    return PathPlanResponse(OutPlan)


def handle_ReachCheck(req):
    print("REACH CHECK RECEIVED")
    Proc=req.ReachIn.Procedure
    iterations=len(req.ReachIn.structList)
    DrillOp=str(req.ReachIn.structList[0].drillOp)
    Outstruct=np.zeros((iterations,9))
    #Take param from server
    if Proc==1:
       HoleAppr=rospy.get_param("/DrillingParam/HoleAppr")
       Max_dist=rospy.get_param("/DrillingParam/MinDist")
       N_dis=rospy.get_param("/DrillingParam/PossRot")
       toolx=rospy.get_param("/DTool_ref"+DrillOp+"/x")
       tooly=rospy.get_param("/DTool_ref"+DrillOp+"/y")
       toolz=rospy.get_param("/DTool_ref"+DrillOp+"/z")
       toolq1=rospy.get_param("/DTool_ref"+DrillOp+"/q1")
       toolq2=rospy.get_param("/DTool_ref"+DrillOp+"/q2")
       toolq3=rospy.get_param("/DTool_ref"+DrillOp+"/q3")
       toolq4=rospy.get_param("/DTool_ref"+DrillOp+"/q4")
    else:
       HoleAppr=0
       Max_dist=rospy.get_param("/DenseScanParam/MinDist")
       N_dis=rospy.get_param("/DenseScanParam/PossRot")
       toolx=rospy.get_param("/STool_ref/x")
       tooly=rospy.get_param("/STool_ref/y")
       toolz=rospy.get_param("/STool_ref/z")
       toolq1=rospy.get_param("/STool_ref/q1")
       toolq2=rospy.get_param("/STool_ref/q2")
       toolq3=rospy.get_param("/STool_ref/q3")
       toolq4=rospy.get_param("/STool_ref/q4")
    dTheta=2*math.pi/N_dis
    woobjx=rospy.get_param("/Wobj_ref/ufx")
    woobjy=rospy.get_param("/Wobj_ref/ufy")
    woobjz=rospy.get_param("/Wobj_ref/ufz")
    tool=[toolx,tooly,toolz,toolq1,toolq2,toolq3,toolq4]
    obj=[woobjx,woobjy,woobjz]
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
    #Get the proper structure
    for i in range(iterations):
        Outstruct[i,0]=req.ReachIn.structList[i].label
        Outstruct[i,1]=req.ReachIn.structList[i].Pose.position.x
        Outstruct[i,2]=req.ReachIn.structList[i].Pose.position.y
        Outstruct[i,3]=req.ReachIn.structList[i].Pose.position.z
        Outstruct[i,4]=req.ReachIn.structList[i].Pose.orientation.w
        Outstruct[i,5]=req.ReachIn.structList[i].Pose.orientation.x
        Outstruct[i,6]=req.ReachIn.structList[i].Pose.orientation.y
        Outstruct[i,7]=req.ReachIn.structList[i].Pose.orientation.z
        Outstruct[i,8]=req.ReachIn.structList[i].reach
    labels=Outstruct[:,0]
    labels_set=set(labels)
    NTraj= len(labels_set) - (1 if -1 in labels else 0)
    if Proc ==0:
        Md=rospy.get_param('/DrillingParam/MinDist')
        Surf_points=np.zeros((len(Outstruct),7))
        for i in range(len(Outstruct)):
            Surf_points[i,:]=AlongZ(Outstruct[i,1:8],-Md)
            #Surf_points[i,:]=AlongZ(Targ_fori[i,:],-Max_dist)
        PopT,Pcov=curve_fit(paraBolEqn,np.vstack((Surf_points[:,0],Surf_points[:,1])),Surf_points[:,2],p0=[1,1,1,1,1], maxfev=1000000)
        rospy.set_param('/DrillingParam/popt0', float(PopT[0]))
        rospy.set_param('/DrillingParam/popt1', float(PopT[1]))
        rospy.set_param('/DrillingParam/popt2', float(PopT[2]))
        rospy.set_param('/DrillingParam/popt3', float(PopT[3]))
        rospy.set_param('/DrillingParam/popt4', float(PopT[4]))
    Outstruct=MaxReach1DoF(Outstruct,labels_set,tool,obj,dh_a,dh_d,dh_alpha,dTheta,Max_dist,Proc,HoleAppr)
    for k in range(iterations):
        req.ReachIn.structList[k].reach=int(Outstruct[k,8])
        req.ReachIn.structList[k].Pose.orientation.w= Outstruct[k,4]
        req.ReachIn.structList[k].Pose.orientation.x= Outstruct[k,5]
        req.ReachIn.structList[k].Pose.orientation.y= Outstruct[k,6]
        req.ReachIn.structList[k].Pose.orientation.z= Outstruct[k,7]
    return ReachCheckResponse(req.ReachIn)


