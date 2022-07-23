#!/usr/bin/env python


import numpy as np
import quaternion
import math
import matplotlib.pyplot as plt
from scipy.spatial.distance import pdist, squareform

####Paraboloid equation
def paraBolEqn(data,a,b,c,d,e):
    x,y = data
    z= -(((x-b)/a)**2+((y-d)/c)**2)+e
    #z=a + b * x**0 * y**0 + c * x**0 * y**1 + d * x**0 * y**2 
    #+ e * x**1 * y**0 + f * x**1 * y**1 + g * x**1 * y**2
    #+ h * x**2 * y**0 + i * x**2 * y**1 + j * x**2 * y**2
    return z


#####Order the target matrix with respect to clusters#####
def Order_from_clust(in_matrix,ord_matrix,label_arr,label_set):
    
    keep_track=0
    for i in label_set:
        finder = np.argwhere(label_arr == i)
        finder_len=len(finder)
        for j in range(finder_len):
            ord_matrix[keep_track+j,0]=i #labels in the first column
            ord_matrix[keep_track+j,1:-1]=in_matrix[finder[j],:]
        keep_track=keep_track+finder_len
    return ord_matrix    

#####Get the target matrix without No reachable targets####
def Remove_no_reach(matrix,reach_column):
    search_noreach=np.argwhere(matrix[:,reach_column]==0)
    cleared_matrix=np.delete(matrix,reach_column,1) ###remove reach column
    cleared_matrix=np.delete(cleared_matrix,search_noreach,0)
    return cleared_matrix

#####Build the matrix containing the extremes of the trajectories###
def Get_Extremes(ord_fori_mat,labelsSet):
    count=-1
    extremes_m=np.zeros((len(labelsSet)*2,4))

    for i in labelsSet:
        finder=np.argwhere(ord_fori_mat[:,0] == i)
        f_len=len(finder)
        supp=np.zeros((f_len,3))
        if f_len > 1 :
            for j in range(f_len):
                supp[j,:]=ord_fori_mat[finder[j],1:4]
            Dist=pdist(supp)
            Z=squareform(Dist)
            max_el=np.max(np.max(Z))
            ii=np.argwhere(Z == max_el)
            count=count+1
            extremes_m[count,0]=ord_fori_mat[finder[ii[0,0]],1]
            extremes_m[count,1]=ord_fori_mat[finder[ii[0,0]],2]
            extremes_m[count,2]=ord_fori_mat[finder[ii[0,0]],3]
            extremes_m[count,3]=ord_fori_mat[finder[ii[0,0]],0]*2
            count=count+1
            extremes_m[count,0]=ord_fori_mat[finder[ii[1,0]],1]
            extremes_m[count,1]=ord_fori_mat[finder[ii[1,0]],2]
            extremes_m[count,2]=ord_fori_mat[finder[ii[1,0]],3]
            extremes_m[count,3]=ord_fori_mat[finder[ii[1,0]],0]*2+1
    cut_ext = np.argwhere(extremes_m[:,2] == 0)
    extremes_m = np.delete(extremes_m, cut_ext, 0)
   

    return extremes_m


#####Generate target along Z####
def AlongZ(In,dist):
    if len(In)==8:
        qw=In[4]
        qx=In[5]
        qy=In[6]
        qz=In[7]
        Rot_m=np.zeros((3,3))
        Rot_m[0][0]=qw**2+qx**2-qy**2-qz**2
        Rot_m[0][1]=2*qx*qy-2*qw*qz
        Rot_m[0][2]=2*qx*qz+2*qy*qw
        Rot_m[1][0]=2*qx*qy+2*qz*qw
        Rot_m[1][1]=qw**2-qx**2+qy**2-qz**2
        Rot_m[1][2]=2*qy*qz-2*qw*qx
        Rot_m[2][0]=2*qx*qz-2*qw*qy
        Rot_m[2][1]=2*qy*qz+2*qw*qx
        Rot_m[2][2]=qw**2-qx**2-qy**2+qz**2
        
        z=[0,0,1]
        vers=[0.0,0.0,0.0]
        vers[0]=Rot_m[0][2]
        vers[1]=Rot_m[1][2]
        vers[2]=Rot_m[2][2]
    
        norm=np.linalg.norm(vers)
        vers=vers/norm
        off_xyz_alz=[0,0,0,0,0,0,0,0]
        off_xyz_alz[0]=In[0]
        off_xyz_alz[1:4]=In[1:4]+dist*vers
        off_xyz_alz[4:]=In[4:]
    elif len(In)==7:
        qw=In[3]
        qx=In[4]
        qy=In[5]
        qz=In[6]
        Rot_m=np.zeros((3,3))
        Rot_m[0][0]=qw**2+qx**2-qy**2-qz**2
        Rot_m[0][1]=2*qx*qy-2*qw*qz
        Rot_m[0][2]=2*qx*qz+2*qy*qw
        Rot_m[1][0]=2*qx*qy+2*qz*qw
        Rot_m[1][1]=qw**2-qx**2+qy**2-qz**2
        Rot_m[1][2]=2*qy*qz-2*qw*qx
        Rot_m[2][0]=2*qx*qz-2*qw*qy
        Rot_m[2][1]=2*qy*qz+2*qw*qx
        Rot_m[2][2]=qw**2-qx**2-qy**2+qz**2
        
        z=[0,0,1]
        vers=[0.0,0.0,0.0]
        vers[0]=Rot_m[0][2]
        vers[1]=Rot_m[1][2]
        vers[2]=Rot_m[2][2]
    
        norm=np.linalg.norm(vers)
        vers=vers/norm
        off_xyz_alz=[0,0,0,0,0,0,0]
        
        off_xyz_alz[0:3]=In[0:3]+dist*vers
        off_xyz_alz[3:]=In[3:]

    return off_xyz_alz

###Retrieve Link6 POse from TCP target

def FromTCPtoLink6(Targ,TCP_l6):
    qw=Targ[3]
    qx=Targ[4]
    qy=Targ[5]
    qz=Targ[6]
    Rot_m=np.zeros((3,3))
    Rot_m[0][0]=qw**2+qx**2-qy**2-qz**2
    Rot_m[0][1]=2*qx*qy-2*qw*qz
    Rot_m[0][2]=2*qx*qz+2*qy*qw
    Rot_m[1][0]=2*qx*qy+2*qz*qw
    Rot_m[1][1]=qw**2-qx**2+qy**2-qz**2
    Rot_m[1][2]=2*qy*qz-2*qw*qx
    Rot_m[2][0]=2*qx*qz-2*qw*qy
    Rot_m[2][1]=2*qy*qz+2*qw*qx
    Rot_m[2][2]=qw**2-qx**2-qy**2+qz**2

    
    qw_t=TCP_l6[3]
    qx_t=TCP_l6[4]
    qy_t=TCP_l6[5]
    qz_t=TCP_l6[6]
    Rot_t=np.zeros((3,3))
    Rot_t[0][0]=qw_t**2+qx_t**2-qy_t**2-qz_t**2
    Rot_t[0][1]=2*qx_t*qy_t-2*qw_t*qz_t
    Rot_t[0][2]=2*qx_t*qz_t+2*qy_t*qw_t
    Rot_t[1][0]=2*qx_t*qy_t+2*qz_t*qw_t
    Rot_t[1][1]=qw_t**2-qx_t**2+qy_t**2-qz_t**2
    Rot_t[1][2]=2*qy_t*qz_t-2*qw_t*qx_t
    Rot_t[2][0]=2*qx_t*qz_t-2*qw_t*qy_t
    Rot_t[2][1]=2*qy_t*qz_t+2*qw_t*qx_t
    Rot_t[2][2]=qw_t**2-qx_t**2-qy_t**2+qz_t**2

    Rot_fin=np.matmul(Rot_m, np.linalg.inv(Rot_t))
    
    off=[0.0,0.0,0.0]
    off[0]=Rot_fin[0][0]*(-TCP_l6[0])+Rot_fin[0][1]*(-TCP_l6[1])+Rot_fin[0][2]*(-TCP_l6[2])
    off[1]=Rot_fin[1][0]*(-TCP_l6[0])+Rot_fin[1][1]*(-TCP_l6[1])+Rot_fin[1][2]*(-TCP_l6[2])
    off[2]=Rot_fin[2][0]*(-TCP_l6[0])+Rot_fin[2][1]*(-TCP_l6[1])+Rot_fin[2][2]*(-TCP_l6[2])

    

    l6=[0.0,0.0,0.0,0.0,0.0,0.0,0.0]
    l6[0]=Targ[0]+ off[0]
    l6[1]=Targ[1]+ off[1]
    l6[2]=Targ[2]+ off[2]

    sy= math.sqrt(Rot_fin[0][0]**2+Rot_fin[1][0]**2)
    singular = sy < 1e-6
    Eu=[0.0,0.0,0.0]
    Eu[1]= math.atan2(-Rot_fin[2][0], sy)
    if singular:
        Eu[0] = 0
        Eu[2] =math.atan2(-Rot_fin[1][2], Rot_fin[1][1])
    else:
        Eu[0] = math.atan2(Rot_fin[1][0], Rot_fin[0][0])
        Eu[2] = math.atan2(Rot_fin[2][1],Rot_fin[2][2])
    

    ca = math.cos(Eu[0]/2)
    sa = math.sin(Eu[0]/2)
    cb = math.cos(Eu[1]/2)
    sb = math.sin(Eu[1]/2)
    cg = math.cos(Eu[2]/2)
    sg = math.sin(Eu[2]/2)

    l6[3]=cg*cb*ca+sg*sb*sa
    l6[4]=sg*cb*ca-cg*sb*sa
    l6[5]=cg*sb*ca+sg*cb*sa
    l6[6]=cg*cb*sa-sg*sb*ca

    return l6





def RotQuat_arZ(qstart,theta):
    w0=qstart[0]
    x0=qstart[1]
    y0=qstart[2]
    z0=qstart[3]

    Rot_fromq=[[0,0,0],[0,0,0],[0,0,0]]
    Rot_fromq[0][0]= w0**2+x0**2-y0**2-z0**2
    Rot_fromq[0][1]= 2*x0*y0-2*w0*z0
    Rot_fromq[0][2]= 2*x0*z0 + 2*w0*y0
    Rot_fromq[1][0]= 2*x0*y0 + 2*w0*z0
    Rot_fromq[1][1]= w0**2-x0**2+y0**2-z0**2
    Rot_fromq[1][2]= 2*y0*z0 -2*w0*x0
    Rot_fromq[2][0]= 2*x0*z0-2*w0*y0
    Rot_fromq[2][1]= 2*y0*z0+2*w0*x0
    Rot_fromq[2][2]= w0**2-x0**2-y0**2+z0**2


    Rot_z=[[0,0,0],[0,0,0],[0,0,0]]
    Rot_z[0][0]=math.cos(theta)
    Rot_z[0][1]= -math.sin(theta)
    Rot_z[0][2]= 0 
    Rot_z[1][0]= math.sin(theta)
    Rot_z[1][1]= math.cos(theta)
    Rot_z[1][2]= 0
    Rot_z[2][0]= 0
    Rot_z[2][1]= 0
    Rot_z[2][2]= 1

    Rot_fromq=np.array(Rot_fromq)
    Rot_z=np.array(Rot_z)

    Rot_fin=np.matmul(Rot_fromq,Rot_z)
    
    
    sy= math.sqrt(Rot_fin[0][0]**2+Rot_fin[1][0]**2)
    singular = sy < 1e-6
    Eu=[0.0,0.0,0.0]
    Eu[1]= math.atan2(-Rot_fin[2][0], sy)
    if singular:
        Eu[0] = 0
        Eu[2] =math.atan2(-Rot_fin[1][2], Rot_fin[1][1])
    else:
        Eu[0] = math.atan2(Rot_fin[1][0], Rot_fin[0][0])
        Eu[2] = math.atan2(Rot_fin[2][1],Rot_fin[2][2])
    

    ca = math.cos(Eu[0]/2)
    sa = math.sin(Eu[0]/2)
    cb = math.cos(Eu[1]/2)
    sb = math.sin(Eu[1]/2)
    cg = math.cos(Eu[2]/2)
    sg = math.sin(Eu[2]/2)

    wfin=cg*cb*ca+sg*sb*sa
    xfin=sg*cb*ca-cg*sb*sa
    yfin=cg*sb*ca+sg*cb*sa
    zfin=cg*cb*sa-sg*sb*ca






    quatnew=[wfin,xfin,yfin,zfin]
    return quatnew


def RotToQuat(R):
    sy= math.sqrt(R[0][0]**2+R[1][0]**2)
    singular = sy < 1e-6
    Eu=[0.0,0.0,0.0]
    Eu[1]= math.atan2(-R[2][0], sy)
    if singular:
        Eu[0] = 0
        Eu[2] =math.atan2(-R[1][2], R[1][1])
    else:
        Eu[0] = math.atan2(R[1][0], R[0][0])
        Eu[2] = math.atan2(R[2][1],R[2][2])
    

    ca = math.cos(Eu[0]/2)
    sa = math.sin(Eu[0]/2)
    cb = math.cos(Eu[1]/2)
    sb = math.sin(Eu[1]/2)
    cg = math.cos(Eu[2]/2)
    sg = math.sin(Eu[2]/2)

    wfin=cg*cb*ca+sg*sb*sa
    xfin=sg*cb*ca-cg*sb*sa
    yfin=cg*sb*ca+sg*cb*sa
    zfin=cg*cb*sa-sg*sb*ca
    quat=[wfin,xfin,yfin,zfin]
    return quat
    


#####Find nearest trajectory from actual robot pose#####
def start_from(Start_point, extremes):
    Alldist=np.zeros((len(extremes),1))
    for g in range(len(extremes)):
        Alldist[g,0]=math.sqrt((Start_point[0]-extremes[g,0])**2+(Start_point[1]-extremes[g,1])**2+(Start_point[2]-extremes[g,2])**2)
    minimum=np.min(np.min(Alldist))
    index=np.min(np.argwhere(Alldist[:,0]==minimum))
    print("index start from :"+str(index))
    return index 
###Interpolation function Cartesian poses####
def interpolate(start, end, t_final, step_size):  #t-f OR=4
    num_steps=int((t_final)/step_size)
    if num_steps<50:
        num_steps=50
    interr_arr=np.linspace(start, end, num_steps)
    t_arr = np.linspace(0, t_final, num_steps)
    return interr_arr


### Raw Interpolate function Joint poses###
def j_int(startj,endj,ang_vel,step_size):
    times=np.zeros(6)
    startj=np.squeeze(startj)
    endj=np.squeeze(endj)
    print(startj)
    print(endj)
    for i in range(6):
        times[i]=abs(endj[i]-startj[i])/ang_vel
    print(times)
    t_final=np.max(np.max(times))
    num_steps=int((t_final)*250)
    if num_steps<50:
        num_steps=50
    j_arr=np.zeros((6,num_steps))
    for t in range(6):
        #start=startj[t]
        #end=endj[t]
        j_arr[t,:]=np.linspace(startj[t],endj[t], num_steps)
       # j_arr[t,:]=j_arr_i
    return j_arr    


#def j_int_trap(startj,endj,ang_vel,step_size):
#    times=np.zeros(6)
#    startj=np.squeeze(startj)
#    endj=np.squeeze(endj)
#    for i in range(6):
#        times[i]=abs(endj[i]-startj[i])/ang_vel
#    t_final=np.max(np.max(times))
#    where=np.argwhere(times==t_final)
#    num_steps=int((t_final)/step_size)
#    j_arr=np.zeros((6,num_steps))
#    for t in range(6):
#        #start=startj[t]
#        #end=endj[t]
#        j_arr[t,:]=np.linspace(startj[t],endj[t], num_steps)
#       # j_arr[t,:]=j_arr_i
#    return j_arr    








def slerp(qinw,qinx,qiny,qinz,qfinw,qfinx,qfiny,qfinz,t_fin,t_arr):
   
    #if qinx*qfinx+qiny*qfiny+qinz*qfinz+qinw*qfinw >= 1 or qinx*qfinx+qiny*qfiny+qinz*qfinz+qinw*qfinw <= -1:
    #    qw=qfinw
    #    qx=qfinx
    #    qy=qfiny
    #    qz=qfinz        
#
#
    #else:
    #    theta = math.acos(qinx*qfinx+qiny*qfiny+qinz*qfinz+qinw*qfinw)
    #    sin = math.sin(theta)
    #    
    #    t=1-T
    #    
    #    v_qin = math.sin(t*theta) / sin
    #    v_qfin = math.sin(T*theta) / sin 
    #    #q_int =np.array([ v_qin*qinw + v_qfin*qfinw, v_qin*qinx + v_qfin*qfinx, v_qin*qiny + v_qfin*qfiny, v_qin*qinz + v_qfin*qfinz]) #w
    #    #norm = np.linalg.norm(q_int)
    #    #q_int = q_int / norm
    #    qx=v_qin*qinx +v_qfin*qfinx
    #    qy=v_qin*qiny +v_qfin*qfiny
    #    qz=v_qin*qinz +v_qfin*qfinz
    #    qw=v_qin*qinw +v_qfin*qfinw
    
    #print(T)
    #print(v_qfin)
    qw=np.zeros(len(t_arr))
    qx=np.zeros(len(t_arr))
    qy=np.zeros(len(t_arr))
    qz=np.zeros(len(t_arr))
    r1=np.quaternion(qinw,qinx,qiny,qinz)
    r2=np.quaternion(qfinw,qfinx,qfiny,qfinz)
    quat=quaternion.slerp(r1,r2,0,t_fin,t_arr)
    for i in range(len(t_arr)):
        qw[i]=quat[i].components[0]
        qx[i]=quat[i].components[1]
        qy[i]=quat[i].components[2]
        qz[i]=quat[i].components[3]
    return  qw ,qx, qy, qz
       





###Velocity function (Test)#####
def Const_vel(x_in, y_in, z_in, x_f, y_f, z_f, vel):
    D = math.sqrt((x_f-x_in)**2 + (y_f-y_in)**2 + (z_f-z_in)**2)
    Tot_time = D / vel 
    return Tot_time