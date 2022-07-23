import numpy as np
import math

from numpy.core.records import array
from numpy.lib.function_base import interp, rot90
from Util import*
from numpy.core.numeric import NaN
from sklearn.cluster import DBSCAN

from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
import scipy as sp
from scipy.optimize import curve_fit

###DH parameters###
q=np.zeros(6)
theta=[q[0],q[1]-math.pi/2,q[2],q[3],q[4]+math.pi,q[5]]
d=[0.495,0,0,0.960,0,0.135] #IRB4600
a=[0.175,0.9,0.175,0,0,0]
alpha=[-math.pi/2,0,-math.pi/2,math.pi/2,math.pi/2,0]

#d=[0.78,0,0,1.1425,0,0.2]   #IRB6700
#a=[0.32,1.125,0.2,0,0,0]
#alpha=[-math.pi/2,0,-math.pi/2,math.pi/2,-math.pi/2,0]
###Joint limits####
min_angle=[math.radians(-180),math.radians(-90),math.radians(-180),math.radians(-400),math.radians(-90),math.radians(-400)]  #IRB4600
max_angle=[math.radians(180),math.radians(150),math.radians(60),math.radians(400),math.radians(90),math.radians(400)]

#min_angle=[math.radians(-170),math.radians(-65),math.radians(-180),math.radians(-300),math.radians(-110),math.radians(-360)]  #IRB4600
#max_angle=[math.radians(170),math.radians(85),math.radians(65),math.radians(300),math.radians(110),math.radians(360)]
###Util functions###

###1) get homogenous transformation matrix
def get_Hom_transf(theta,d,a,alpha,ind):
    i=ind
    theta=theta[i]
    d=d[i]
    a=a[i]
    alpha=alpha[i]
    A=np.zeros((4,4))
    A[0,:]=[math.cos(theta), -math.cos(alpha)*math.sin(theta), math.sin(alpha)*math.sin(theta),a*math.cos(theta)]
    A[1,:]=[math.sin(theta), math.cos(theta)*math.cos(alpha), -math.sin(alpha)*math.cos(theta), a*math.sin(theta)]
    A[2,:]=[0,math.sin(alpha),math.cos(alpha),d]
    A[3,3]=1
    return A

###2)Solve spherical wrist for q[3],q5,q6



#######Get spherical wrist solution to obtain joints4-5-6->EulerZYZ#######
def spherical_wrist_sol_drill(q1_3,d,a,alpha,T,wrist):
    q4_6=np.zeros(3)
    T01=[[math.cos(q1_3[0]), -math.sin(q1_3[0])*math.cos(alpha[0]),math.sin(q1_3[0])*math.sin(alpha[0]),a[0]*math.cos(q1_3[0])],[math.sin(q1_3[0]),math.cos(q1_3[0])*math.cos(alpha[0]),-math.cos(q1_3[0])*math.sin(alpha[0]),a[0]*math.sin(q1_3[0])],[0,math.sin(alpha[0]),math.cos(alpha[0]),d[0]],[0,0,0,1]]
    T12=[[math.cos(q1_3[1]), -math.sin(q1_3[1])*math.cos(alpha[1]),math.sin(q1_3[1])*math.sin(alpha[1]),a[1]*math.cos(q1_3[1])],[math.sin(q1_3[1]),math.cos(q1_3[1])*math.cos(alpha[1]),-math.cos(q1_3[1])*math.sin(alpha[1]),a[1]*math.sin(q1_3[1])],[0,math.sin(alpha[1]),math.cos(alpha[1]),d[1]],[0,0,0,1]]
    T23=[[math.cos(q1_3[2]), -math.sin(q1_3[2])*math.cos(alpha[2]),math.sin(q1_3[2])*math.sin(alpha[2]),a[2]*math.cos(q1_3[2])],[math.sin(q1_3[2]),math.cos(q1_3[2])*math.cos(alpha[2]),-math.cos(q1_3[2])*math.sin(alpha[2]),a[2]*math.sin(q1_3[2])],[0,math.sin(alpha[2]),math.cos(alpha[2]),d[2]],[0,0,0,1]]

    T02=np.matmul(T01,T12)
    T03=np.matmul(T02,T23)

    R03=np.zeros((3,3))
    Re=np.zeros((3,3))

    R03=T03[0:3,0:3]
    Re=T[0:3,0:3]

    R03t=np.transpose(R03)
    R36=np.matmul(R03t,Re)

    thresh=1e-12         #1e-12
    if abs(R36[2,2]-1)>thresh:
        if wrist==1:
            q4_6[0]=math.atan2(R36[1,2],R36[0,2])
            q4_6[0]=check_joint_lim(q4_6[0],min_angle[3],max_angle[3])
            q4_6[1]=math.atan2(-math.sqrt(R36[0,2]**2+R36[1,2]**2),R36[2,2])
            q4_6[1]=check_joint_lim(q4_6[1],min_angle[4],max_angle[4])
            q4_6[2]=math.atan2(-R36[2,1],R36[2,0])
            q4_6[2]=check_joint_lim(q4_6[2],min_angle[5],max_angle[5])
        else:
            q4_6[0]=math.atan2(-R36[1,2],-R36[0,2])
            q4_6[0]=check_joint_lim(q4_6[0],min_angle[3],max_angle[3])
            q4_6[1]=math.atan2(math.sqrt(R36[0,2]**2+R36[1,2]**2),R36[2,2])
            q4_6[1]=check_joint_lim(q4_6[1],min_angle[4],max_angle[4])
            q4_6[2]=math.atan2(R36[2,1],-R36[2,0])
            q4_6[2]=check_joint_lim(q4_6[2],min_angle[5],max_angle[5])
    else: #deg case
        #if wrist==1:
        #    q4_6[0]=0
        #    q4_6[1]=0
        #    q4_6[2]=math.atan2(R36[0,1]-R36[1,0],-R36[0,0]-R36[1,1])
        #    q4_6[2]=check_joint_lim(q4_6[2],min_angle[5],max_angle[5])
        #else:
        #    q4_6[0]=-math.pi
        #    q4_6[1]=0
        #    q4_6[2]=math.atan2(R36[0,1]-R36[1,0],-R36[0,0]-R36[1,1]+math.pi)
        #    q4_6[2]=check_joint_lim(q4_6[2],min_angle[5],max_angle[5])
        q4_6[0]=NaN
        q4_6[1]=NaN
        q4_6[2]=NaN


    return q4_6

def spherical_wrist_sol_scan(q1_3,d,a,alpha,T,wrist):
    q4_6=np.zeros(3)
    T01=[[math.cos(q1_3[0]), -math.sin(q1_3[0])*math.cos(alpha[0]),math.sin(q1_3[0])*math.sin(alpha[0]),a[0]*math.cos(q1_3[0])],[math.sin(q1_3[0]),math.cos(q1_3[0])*math.cos(alpha[0]),-math.cos(q1_3[0])*math.sin(alpha[0]),a[0]*math.sin(q1_3[0])],[0,math.sin(alpha[0]),math.cos(alpha[0]),d[0]],[0,0,0,1]]
    T12=[[math.cos(q1_3[1]), -math.sin(q1_3[1])*math.cos(alpha[1]),math.sin(q1_3[1])*math.sin(alpha[1]),a[1]*math.cos(q1_3[1])],[math.sin(q1_3[1]),math.cos(q1_3[1])*math.cos(alpha[1]),-math.cos(q1_3[1])*math.sin(alpha[1]),a[1]*math.sin(q1_3[1])],[0,math.sin(alpha[1]),math.cos(alpha[1]),d[1]],[0,0,0,1]]
    T23=[[math.cos(q1_3[2]), -math.sin(q1_3[2])*math.cos(alpha[2]),math.sin(q1_3[2])*math.sin(alpha[2]),a[2]*math.cos(q1_3[2])],[math.sin(q1_3[2]),math.cos(q1_3[2])*math.cos(alpha[2]),-math.cos(q1_3[2])*math.sin(alpha[2]),a[2]*math.sin(q1_3[2])],[0,math.sin(alpha[2]),math.cos(alpha[2]),d[2]],[0,0,0,1]]

    T02=np.matmul(T01,T12)
    T03=np.matmul(T02,T23)

    R03=np.zeros((3,3))
    Re=np.zeros((3,3))

    R03=T03[0:3,0:3]
    Re=T[0:3,0:3]

    R03t=np.transpose(R03)
    R36=np.matmul(R03t,Re)

    thresh=1e-12         #1e-12
    if abs(R36[2,2]-1)>thresh:
        if wrist==1:
            q4_6[0]=math.atan2(R36[1,2],R36[0,2])
            q4_6[0]=check_joint_lim(q4_6[0],min_angle[3],max_angle[3])
            q4_6[1]=math.atan2(-math.sqrt(R36[0,2]**2+R36[1,2]**2),R36[2,2])
            q4_6[1]=check_joint_lim(q4_6[1],min_angle[4],max_angle[4])
            q4_6[2]=math.atan2(-R36[2,1],R36[2,0])
            q4_6[2]=check_joint_lim(q4_6[2],min_angle[5],max_angle[5])
        else:
            q4_6[0]=math.atan2(-R36[1,2],-R36[0,2])
            q4_6[0]=check_joint_lim(q4_6[0],min_angle[3],max_angle[3])
            q4_6[1]=math.atan2(math.sqrt(R36[0,2]**2+R36[1,2]**2),R36[2,2])
            q4_6[1]=check_joint_lim(q4_6[1],min_angle[4],max_angle[4])
            q4_6[2]=math.atan2(R36[2,1],-R36[2,0])
            q4_6[2]=check_joint_lim(q4_6[2],min_angle[5],max_angle[5])
    else: #deg case
        if wrist==1:
            q4_6[0]=0
            q4_6[1]=0
            q4_6[2]=math.atan2(R36[0,1]-R36[1,0],-R36[0,0]-R36[1,1])
            q4_6[2]=check_joint_lim(q4_6[2],min_angle[5],max_angle[5])
        else:
            q4_6[0]=-math.pi
            q4_6[1]=0
            q4_6[2]=math.atan2(R36[0,1]-R36[1,0],-R36[0,0]-R36[1,1]+math.pi)
            q4_6[2]=check_joint_lim(q4_6[2],min_angle[5],max_angle[5])
        


    return q4_6


####Check joint limit
def check_joint_lim(joint_value,l_lim,u_lim):
    if l_lim<=joint_value<=u_lim:
        return joint_value
    else:
        new_joint_value = 100
        if joint_value < l_lim:
            new_joint_value=joint_value + 2*math.pi
        else:
            new_joint_value=joint_value - 2*math.pi
        if l_lim < new_joint_value < u_lim :
            return new_joint_value       
        else:
            return NaN


####Build Homegenous matrix T from robot EE pose
def buildT(L6):
    T=np.zeros((4,4))
    
    w0=L6[3]
    x0=L6[4]
    y0=L6[5]
    z0=L6[6]

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

    T[0:3,0:3]=Rot_fromq
    T[0,3]=L6[0]
    T[1,3]=L6[1]
    T[2,3]=L6[2]
    T[3,3]=1
    return T



#### Compute joint2 and joint 3 
def get_joint2(q,d,a,alpha,Pm):
    q2=np.zeros(2)
    L2=a[1]
    L3=math.sqrt(d[3]**2+a[2]**2)
    p1=np.zeros(4)

    A01=get_Hom_transf(q,d,a,alpha,0)
    A01_inv=np.linalg.inv(A01)
    p1[0]=A01_inv[0,0]*Pm[0]+A01_inv[0,1]*Pm[1]+A01_inv[0,2]*Pm[2]+A01_inv[0,3]
    p1[1]=A01_inv[1,0]*Pm[0]+A01_inv[1,1]*Pm[1]+A01_inv[1,2]*Pm[2]+A01_inv[1,3]



    r=math.sqrt(p1[0]**2+p1[1]**2)
    beta=math.atan2(-p1[1],p1[0])
    gamma_arg=(L2**2+r**2-L3**2)/(2*r*L2)

    if gamma_arg<-1 or gamma_arg>1:
        q2[0]=NaN
        q2[1]=NaN
    else:
        gamma=math.acos(gamma_arg)
        q2[0]=math.pi/2- beta - gamma # e up
        q2[1]=math.pi/2-beta + gamma # e down
    return q2    


def get_joint3(q,d,a,alpha,Pm):
    q3=np.zeros(2)
    
    L2=a[1]
    L3=d[3]
    A2=a[2]

    L4=math.sqrt(A2**2+L3**2)
    phi=math.acos((A2**2+L4**2-L3**2)/(2*A2*L4))
    p1=np.zeros(4)

    A01=get_Hom_transf(q,d,a,alpha,0)
    A01_inv=np.linalg.inv(A01)
    p1[0]=A01_inv[0,0]*Pm[0]+A01_inv[0,1]*Pm[1]+A01_inv[0,2]*Pm[2]+A01_inv[0,3]
    p1[1]=A01_inv[1,0]*Pm[0]+A01_inv[1,1]*Pm[1]+A01_inv[1,2]*Pm[2]+A01_inv[1,3]
    r=math.sqrt(p1[0]**2+p1[1]**2)
    eta_arg= (L2**2+L4**2-r**2)/(2*L2*L4)
    if eta_arg<-1 or eta_arg>1:
        q3[0]=NaN
        q3[1]=NaN
    else:
        eta=np.real(math.acos(eta_arg))
    
        q3[0]=math.pi-phi-eta
        q3[1]=math.pi-phi+eta
    return q3


######Compute Jacobian matrix#######
def get_Jac(q,d,a,alpha):
    n_DOF=6
    z0=np.zeros(3)
    z0[2]=1
    z_i=np.zeros((3,6)) #zi vectors
    dp=np.zeros((3,6)) #p-pi-1
    J=np.zeros((6,6))
    T6=Dir_kin_irb4600(a,d,alpha,q,6)
    p=T6[0:3,3]
    #get zi
    for i in range(n_DOF):
        if i==0:
            J[0:3,i]=np.cross(z0,p)
            J[3:6,i]=z0
        else:    

       
            p_i=Dir_kin_irb4600(a,d,alpha,q,i)
            dp[0,i]=p[0]-p_i[0][3]
            dp[1,i]=p[1]-p_i[1][3]
            dp[2,i]=p[2]-p_i[2][3]
            z_i[0,i]=p_i[0][2]
            z_i[1,i]=p_i[1][2]
            z_i[2,i]=p_i[2][2]

            J[0:3,i]=np.cross(z_i[:,i],dp[:,i])
            J[3:6,i]=z_i[:,i]
    return J



####Direct kinematics######
def Dir_kin_irb4600(a,d,alpha,q,outlink,tool):
    th2=q[1]-math.pi/2
    th5=q[4]+math.pi
    T=np.eye(4,4)
    

    if outlink==0:
        o=[0,0,0]
        return o
    elif outlink==1:
        
        T01=[[math.cos(q[0]), -math.sin(q[0])*math.cos(alpha[0]),math.sin(q[0])*math.sin(alpha[0]),a[0]*math.cos(q[0])],[math.sin(q[0]),math.cos(q[0])*math.cos(alpha[0]),-math.cos(q[0])*math.sin(alpha[0]),a[0]*math.sin(q[0])],[0,math.sin(alpha[0]),math.cos(alpha[0]),d[0]],[0,0,0,1]]
        return T01

    elif outlink==2:
        T01=[[math.cos(q[0]), -math.sin(q[0])*math.cos(alpha[0]),math.sin(q[0])*math.sin(alpha[0]),a[0]*math.cos(q[0])],[math.sin(q[0]),math.cos(q[0])*math.cos(alpha[0]),-math.cos(q[0])*math.sin(alpha[0]),a[0]*math.sin(q[0])],[0,math.sin(alpha[0]),math.cos(alpha[0]),d[0]],[0,0,0,1]]
        T12=[[math.cos(th2), -math.sin(th2)*math.cos(alpha[1]),math.sin(th2)*math.sin(alpha[1]),a[1]*math.cos(th2)],[math.sin(th2),math.cos(th2)*math.cos(alpha[1]),-math.cos(th2)*math.sin(alpha[1]),a[1]*math.sin(th2)],[0,math.sin(alpha[1]),math.cos(alpha[1]),d[1]],[0,0,0,1]]
        T02=np.matmul(T01,T12)
        return T02
    elif outlink==3:
        T01=[[math.cos(q[0]), -math.sin(q[0])*math.cos(alpha[0]),math.sin(q[0])*math.sin(alpha[0]),a[0]*math.cos(q[0])],[math.sin(q[0]),math.cos(q[0])*math.cos(alpha[0]),-math.cos(q[0])*math.sin(alpha[0]),a[0]*math.sin(q[0])],[0,math.sin(alpha[0]),math.cos(alpha[0]),d[0]],[0,0,0,1]]
        T12=[[math.cos(th2), -math.sin(th2)*math.cos(alpha[1]),math.sin(th2)*math.sin(alpha[1]),a[1]*math.cos(th2)],[math.sin(th2),math.cos(th2)*math.cos(alpha[1]),-math.cos(th2)*math.sin(alpha[1]),a[1]*math.sin(th2)],[0,math.sin(alpha[1]),math.cos(alpha[1]),d[1]],[0,0,0,1]]
        T23=[[math.cos(q[2]), -math.sin(q[2])*math.cos(alpha[2]),math.sin(q[2])*math.sin(alpha[2]),a[2]*math.cos(q[2])],[math.sin(q[2]),math.cos(q[2])*math.cos(alpha[2]),-math.cos(q[2])*math.sin(alpha[2]),a[2]*math.sin(q[2])],[0,math.sin(alpha[2]),math.cos(alpha[2]),d[2]],[0,0,0,1]]
        T02=np.matmul(T01,T12)
        T03=np.matmul(T02,T23)
        return T03
    elif outlink==4:
        T01=[[math.cos(q[0]), -math.sin(q[0])*math.cos(alpha[0]),math.sin(q[0])*math.sin(alpha[0]),a[0]*math.cos(q[0])],[math.sin(q[0]),math.cos(q[0])*math.cos(alpha[0]),-math.cos(q[0])*math.sin(alpha[0]),a[0]*math.sin(q[0])],[0,math.sin(alpha[0]),math.cos(alpha[0]),d[0]],[0,0,0,1]]
        T12=[[math.cos(th2), -math.sin(th2)*math.cos(alpha[1]),math.sin(th2)*math.sin(alpha[1]),a[1]*math.cos(th2)],[math.sin(th2),math.cos(th2)*math.cos(alpha[1]),-math.cos(th2)*math.sin(alpha[1]),a[1]*math.sin(th2)],[0,math.sin(alpha[1]),math.cos(alpha[1]),d[1]],[0,0,0,1]]
        T23=[[math.cos(q[2]), -math.sin(q[2])*math.cos(alpha[2]),math.sin(q[2])*math.sin(alpha[2]),a[2]*math.cos(q[2])],[math.sin(q[2]),math.cos(q[2])*math.cos(alpha[2]),-math.cos(q[2])*math.sin(alpha[2]),a[2]*math.sin(q[2])],[0,math.sin(alpha[2]),math.cos(alpha[2]),d[2]],[0,0,0,1]]
        T02=np.matmul(T01,T12)
        T03=np.matmul(T02,T23)
        T34=[[math.cos(q[3]), -math.sin(q[3])*math.cos(alpha[3]),math.sin(q[3])*math.sin(alpha[3]),a[3]*math.cos(q[3])],[math.sin(q[3]),math.cos(q[3])*math.cos(alpha[3]),-math.cos(q[3])*math.sin(alpha[3]),a[3]*math.sin(q[3])],[0,math.sin(alpha[3]),math.cos(alpha[3]),d[3]],[0,0,0,1]]
        T04=np.matmul(T03,T34)
        return T04
    elif outlink==5:
        T01=[[math.cos(q[0]), -math.sin(q[0])*math.cos(alpha[0]),math.sin(q[0])*math.sin(alpha[0]),a[0]*math.cos(q[0])],[math.sin(q[0]),math.cos(q[0])*math.cos(alpha[0]),-math.cos(q[0])*math.sin(alpha[0]),a[0]*math.sin(q[0])],[0,math.sin(alpha[0]),math.cos(alpha[0]),d[0]],[0,0,0,1]]
        T12=[[math.cos(th2), -math.sin(th2)*math.cos(alpha[1]),math.sin(th2)*math.sin(alpha[1]),a[1]*math.cos(th2)],[math.sin(th2),math.cos(th2)*math.cos(alpha[1]),-math.cos(th2)*math.sin(alpha[1]),a[1]*math.sin(th2)],[0,math.sin(alpha[1]),math.cos(alpha[1]),d[1]],[0,0,0,1]]
        T23=[[math.cos(q[2]), -math.sin(q[2])*math.cos(alpha[2]),math.sin(q[2])*math.sin(alpha[2]),a[2]*math.cos(q[2])],[math.sin(q[2]),math.cos(q[2])*math.cos(alpha[2]),-math.cos(q[2])*math.sin(alpha[2]),a[2]*math.sin(q[2])],[0,math.sin(alpha[2]),math.cos(alpha[2]),d[2]],[0,0,0,1]]
        T02=np.matmul(T01,T12)
        T03=np.matmul(T02,T23)
        T34=[[math.cos(q[3]), -math.sin(q[3])*math.cos(alpha[3]),math.sin(q[3])*math.sin(alpha[3]),a[3]*math.cos(q[3])],[math.sin(q[3]),math.cos(q[3])*math.cos(alpha[3]),-math.cos(q[3])*math.sin(alpha[3]),a[3]*math.sin(q[3])],[0,math.sin(alpha[3]),math.cos(alpha[3]),d[3]],[0,0,0,1]]
        T04=np.matmul(T03,T34)
        T45=[[math.cos(th5), -math.sin(th5)*math.cos(alpha[4]),math.sin(th5)*math.sin(alpha[4]),a[4]*math.cos(th5)],[math.sin(th5),math.cos(th5)*math.cos(alpha[4]),-math.cos(th5)*math.sin(alpha[4]),a[4]*math.sin(th5)],[0,math.sin(alpha[4]),math.cos(alpha[4]),d[4]],[0,0,0,1]]
        T05=np.matmul(T04,T45)
        return T05
    
    elif outlink==6:
        T01=[[math.cos(q[0]), -math.sin(q[0])*math.cos(alpha[0]),math.sin(q[0])*math.sin(alpha[0]),a[0]*math.cos(q[0])],[math.sin(q[0]),math.cos(q[0])*math.cos(alpha[0]),-math.cos(q[0])*math.sin(alpha[0]),a[0]*math.sin(q[0])],[0,math.sin(alpha[0]),math.cos(alpha[0]),d[0]],[0,0,0,1]]
        T12=[[math.cos(th2), -math.sin(th2)*math.cos(alpha[1]),math.sin(th2)*math.sin(alpha[1]),a[1]*math.cos(th2)],[math.sin(th2),math.cos(th2)*math.cos(alpha[1]),-math.cos(th2)*math.sin(alpha[1]),a[1]*math.sin(th2)],[0,math.sin(alpha[1]),math.cos(alpha[1]),d[1]],[0,0,0,1]]
        T23=[[math.cos(q[2]), -math.sin(q[2])*math.cos(alpha[2]),math.sin(q[2])*math.sin(alpha[2]),a[2]*math.cos(q[2])],[math.sin(q[2]),math.cos(q[2])*math.cos(alpha[2]),-math.cos(q[2])*math.sin(alpha[2]),a[2]*math.sin(q[2])],[0,math.sin(alpha[2]),math.cos(alpha[2]),d[2]],[0,0,0,1]]
        T02=np.matmul(T01,T12)
        T03=np.matmul(T02,T23)
        T34=[[math.cos(q[3]), -math.sin(q[3])*math.cos(alpha[3]),math.sin(q[3])*math.sin(alpha[3]),a[3]*math.cos(q[3])],[math.sin(q[3]),math.cos(q[3])*math.cos(alpha[3]),-math.cos(q[3])*math.sin(alpha[3]),a[3]*math.sin(q[3])],[0,math.sin(alpha[3]),math.cos(alpha[3]),d[3]],[0,0,0,1]]
        T04=np.matmul(T03,T34)
        T45=[[math.cos(th5), -math.sin(th5)*math.cos(alpha[4]),math.sin(th5)*math.sin(alpha[4]),a[4]*math.cos(th5)],[math.sin(th5),math.cos(th5)*math.cos(alpha[4]),-math.cos(th5)*math.sin(alpha[4]),a[4]*math.sin(th5)],[0,math.sin(alpha[4]),math.cos(alpha[4]),d[4]],[0,0,0,1]]
        T05=np.matmul(T04,T45)
        T56=[[math.cos(q[5]), -math.sin(q[5])*math.cos(alpha[5]),math.sin(q[5])*math.sin(alpha[5]),a[5]*math.cos(q[5])],[math.sin(q[5]),math.cos(q[5])*math.cos(alpha[5]),-math.cos(q[5])*math.sin(alpha[5]),a[5]*math.sin(q[5])],[0,math.sin(alpha[5]),math.cos(alpha[5]),d[5]],[0,0,0,1]]
        T06=np.matmul(T05,T56)

        return T06
    elif outlink==7:
        T01=[[math.cos(q[0]), -math.sin(q[0])*math.cos(alpha[0]),math.sin(q[0])*math.sin(alpha[0]),a[0]*math.cos(q[0])],[math.sin(q[0]),math.cos(q[0])*math.cos(alpha[0]),-math.cos(q[0])*math.sin(alpha[0]),a[0]*math.sin(q[0])],[0,math.sin(alpha[0]),math.cos(alpha[0]),d[0]],[0,0,0,1]]
        T12=[[math.cos(th2), -math.sin(th2)*math.cos(alpha[1]),math.sin(th2)*math.sin(alpha[1]),a[1]*math.cos(th2)],[math.sin(th2),math.cos(th2)*math.cos(alpha[1]),-math.cos(th2)*math.sin(alpha[1]),a[1]*math.sin(th2)],[0,math.sin(alpha[1]),math.cos(alpha[1]),d[1]],[0,0,0,1]]
        T23=[[math.cos(q[2]), -math.sin(q[2])*math.cos(alpha[2]),math.sin(q[2])*math.sin(alpha[2]),a[2]*math.cos(q[2])],[math.sin(q[2]),math.cos(q[2])*math.cos(alpha[2]),-math.cos(q[2])*math.sin(alpha[2]),a[2]*math.sin(q[2])],[0,math.sin(alpha[2]),math.cos(alpha[2]),d[2]],[0,0,0,1]]
        T02=np.matmul(T01,T12)
        T03=np.matmul(T02,T23)
        T34=[[math.cos(q[3]), -math.sin(q[3])*math.cos(alpha[3]),math.sin(q[3])*math.sin(alpha[3]),a[3]*math.cos(q[3])],[math.sin(q[3]),math.cos(q[3])*math.cos(alpha[3]),-math.cos(q[3])*math.sin(alpha[3]),a[3]*math.sin(q[3])],[0,math.sin(alpha[3]),math.cos(alpha[3]),d[3]],[0,0,0,1]]
        T04=np.matmul(T03,T34)
        T45=[[math.cos(th5), -math.sin(th5)*math.cos(alpha[4]),math.sin(th5)*math.sin(alpha[4]),a[4]*math.cos(th5)],[math.sin(th5),math.cos(th5)*math.cos(alpha[4]),-math.cos(th5)*math.sin(alpha[4]),a[4]*math.sin(th5)],[0,math.sin(alpha[4]),math.cos(alpha[4]),d[4]],[0,0,0,1]]
        T05=np.matmul(T04,T45)
        T56=[[math.cos(q[5]), -math.sin(q[5])*math.cos(alpha[5]),math.sin(q[5])*math.sin(alpha[5]),a[5]*math.cos(q[5])],[math.sin(q[5]),math.cos(q[5])*math.cos(alpha[5]),-math.cos(q[5])*math.sin(alpha[5]),a[5]*math.sin(q[5])],[0,math.sin(alpha[5]),math.cos(alpha[5]),d[5]],[0,0,0,1]]
        T06=np.matmul(T05,T56)

        Rot_t=np.zeros((3,3))
        Rot_t[0][0]=T06[0,0]
        Rot_t[0][1]=T06[0,1]
        Rot_t[0][2]=T06[0,2]
        Rot_t[1][0]=T06[1,0]
        Rot_t[1][1]=T06[1,1]
        Rot_t[1][2]=T06[1,2]
        Rot_t[2][0]=T06[2,0]
        Rot_t[2][1]=T06[2,1]
        Rot_t[2][2]=T06[2,2]
        
        w0=tool[3]
        x0=tool[4]
        y0=tool[5]
        z0=tool[6]
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

        RotFin=np.matmul(Rot_t,Rot_fromq)
        sy= math.sqrt(RotFin[0][0]**2+RotFin[1][0]**2)
        singular = sy < 1e-6
        Eu=[0.0,0.0,0.0]
        Eu[1]= math.atan2(-RotFin[2][0], sy)
        if singular:
            Eu[0] = 0
            Eu[2] =math.atan2(-RotFin[1][2], RotFin[1][1])
        else:
            Eu[0] = math.atan2(RotFin[1][0], RotFin[0][0])
            Eu[2] = math.atan2(RotFin[2][1],RotFin[2][2])
        
    
        ca = math.cos(Eu[0]/2)
        sa = math.sin(Eu[0]/2)
        cb = math.cos(Eu[1]/2)
        sb = math.sin(Eu[1]/2)
        cg = math.cos(Eu[2]/2)
        sg = math.sin(Eu[2]/2)
        tool_tar=np.zeros(7)
        tool_tar[3]=cg*cb*ca+sg*sb*sa
        tool_tar[4]=sg*cb*ca-cg*sb*sa
        tool_tar[5]=cg*sb*ca+sg*cb*sa
        tool_tar[6]=cg*cb*sa-sg*sb*ca
        tool_tar[0]=T06[0][3]+tool[0]/1000
        tool_tar[1]=T06[1][3]+tool[1]/1000
        tool_tar[2]=T06[2][3]+tool[2]/1000

        return tool_tar

    else:
        return ("Error link request")
















######Main Function IK##############################################################
def IK_irb4600(d,a,alpha,targ_L6, case):
    joints=np.zeros((6,8))
    T=buildT(targ_L6)

    L1=d[0]
    L2=a[1]
    L3=d[3]
    L6=d[5]

    A1=a[0]

    Px=T[0,3]
    Py=T[1,3]
    Pz=T[2,3]

    W= T[0:3,2]

    Pm= np.zeros(3)
    Pm[0]=Px-L6*W[0]
    Pm[1]=Py-L6*W[1]
    Pm[2]=Pz-L6*W[2]

    q1=np.zeros(2)

    q1[0]=math.atan2(Pm[1],Pm[0])
    q1[1]=q1[0]+ math.pi
    #Check limit for q1
    q1[0]=check_joint_lim(q1[0],min_angle[0],max_angle[0])
    q1[1]=check_joint_lim(q1[1],min_angle[0],max_angle[0])

    q2_1=np.zeros(2)
    q2_2=np.zeros(2)

    q3_1=np.zeros(2)
    q3_2=np.zeros(2)

    q2_1=get_joint2([q1[0],0,0,0,0,0],d,a,alpha,Pm)
    q2_2=get_joint2([q1[1],0,0,0,0,0],d,a,alpha,Pm)
    ###Check limit for q2
    q2_1[0]=check_joint_lim(q2_1[0],min_angle[1],max_angle[1])
    q2_1[1]=check_joint_lim(q2_1[1],min_angle[1],max_angle[1])
    q2_2[0]=check_joint_lim(q2_2[0],min_angle[1],max_angle[1])
    q2_2[1]=check_joint_lim(q2_2[1],min_angle[1],max_angle[1])

    q3_1=get_joint3([q1[0],0,0,0,0,0],d,a,alpha,Pm)
    q3_2=get_joint3([q1[1],0,0,0,0,0],d,a,alpha,Pm)
    ###Check limit for q3
    q3_1[0]=check_joint_lim(q3_1[0],min_angle[2],max_angle[2])
    q3_1[1]=check_joint_lim(q3_1[1],min_angle[2],max_angle[2])
    q3_2[0]=check_joint_lim(q3_2[0],min_angle[2],max_angle[2])
    q3_2[1]=check_joint_lim(q3_2[1],min_angle[2],max_angle[2])


    joints[0,:]=[q1[0],q1[0],q1[0],q1[0],q1[1],q1[1],q1[1],q1[1]]
    joints[1,:]=[q2_1[0],q2_1[0],q2_1[1],q2_1[1],q2_2[0],q2_2[0],q2_2[1],q2_2[1]]
    joints[2,:]=[q3_1[0],q3_1[0],q3_1[1],q3_1[1],q3_2[0],q3_2[0],q3_2[1],q3_2[1]]

    joints=np.real(joints)

    #joints[0,:]=np.linalg.norm(joints[0,:])
    #joints[1,:]=np.linalg.norm(joints[1,:])
    

    for i in range(0,8,2):
        q_first=[joints[0,i],joints[1,i]-math.pi/2,joints[2,i]]
        q_second=[joints[0,i+1],joints[1,i+1]-math.pi/2,joints[2,i+1]]
        if joints[1,i]==NaN or joints[2,i]==NaN:
            joints[:,i]=NaN
        else:
            
            if case==1:
                joints[3:7,i]=spherical_wrist_sol_drill(q_first,d,a,alpha,T, 1)
                #jtemp=np.linalg.norm(jtemp)
                
            
                joints[3:7,i+1]=spherical_wrist_sol_drill(q_second,d,a,alpha,T,-1)
                #jtemp=np.linalg.norm(jtemp)
            else:
                joints[3:7,i]=spherical_wrist_sol_scan(q_first,d,a,alpha,T, 1)
                #jtemp=np.linalg.norm(jtemp)
                
            
                joints[3:7,i+1]=spherical_wrist_sol_scan(q_second,d,a,alpha,T,-1)
                #jtemp=np.linalg.norm(jtemp)

    return joints



###if j2j3out=1 In j5 coordinates else j6 coordinates
def IK_irb4600_Red(d,a,alpha,targ,j2j3out,case):  
    joints=np.zeros((6,2))
    Pm= np.zeros(3) 
    L1=d[0]
    L2=a[1]
    L3=d[3]
    L6=d[5]

    A1=a[0]
    if j2j3out==1:
        Pm[0]=targ[0]
        Pm[1]=targ[1]
        Pm[2]=targ[2]
    else:
        T=buildT(targ)

   

        Px=T[0,3]
        Py=T[1,3]
        Pz=T[2,3]
    
        W= T[0:3,2]
    
        
        Pm[0]=Px-L6*W[0]
        Pm[1]=Py-L6*W[1]
        Pm[2]=Pz-L6*W[2]

    

    q1=math.atan2(Pm[1],Pm[0])
    
    #Check limit for q1
    q1=check_joint_lim(q1,min_angle[0],max_angle[0])
   


  

    q2_1=get_joint2([q1,0,0,0,0,0],d,a,alpha,Pm)
   
    ###Check limit for q2
    q2_1[0]=check_joint_lim(q2_1[0],min_angle[1],max_angle[1])
    
   

    q3_1=get_joint3([q1,0,0,0,0,0],d,a,alpha,Pm)
    
    ###Check limit for q3
    q3_1[0]=check_joint_lim(q3_1[0],min_angle[2],max_angle[2])

  


    joints[0,:]=[q1,q1]
    joints[1,:]=[q2_1[0],q2_1[0]]
    joints[2,:]=[q3_1[0],q3_1[0]]

    joints=np.real(joints)
    if j2j3out==1:
        joints13=[0,0,0]
        joints13[0]=q1
        joints13[1]=q2_1[0]
        joints13[2]=q3_1[0]
        return joints13
    #joints[0,:]=np.linalg.norm(joints[0,:])
    #joints[1,:]=np.linalg.norm(joints[1,:])
    else:

        for i in range(0,2,2):
            q_first=[joints[0,i],joints[1,i]-math.pi/2,joints[2,i]]
            q_second=[joints[0,i+1],joints[1,i+1]-math.pi/2,joints[2,i+1]]
            if joints[1,i]==NaN or joints[2,i]==NaN:
                joints[:,i]=NaN
            else:
                
                
                if case==1:
                    joints[3:7,i]=spherical_wrist_sol_drill(q_first,d,a,alpha,T, 1)
                    #jtemp=np.linalg.norm(jtemp)
                    
                
                    joints[3:7,i+1]=spherical_wrist_sol_drill(q_second,d,a,alpha,T,-1)
                    #jtemp=np.linalg.norm(jtemp)
                else:
                    joints[3:7,i]=spherical_wrist_sol_scan(q_first,d,a,alpha,T, 1)
                    #jtemp=np.linalg.norm(jtemp)
                    
                
                    joints[3:7,i+1]=spherical_wrist_sol_scan(q_second,d,a,alpha,T,-1)
                    #jtemp=np.linalg.norm(jtemp)
                
    
        return joints

###Reachability check 
def Reach_check(a,d,alpha,TargL6,proc):
    sol=IK_irb4600(d,a,alpha,TargL6,proc)
    
    check=-1
    for k in range (4):
        arr_sum=np.sum(sol[:,k])
        array_has_nan= np.isnan(arr_sum)
        if array_has_nan == True:
            check=0
            
        else:
            check=1
            break
    return check

def Reach_check_red(a,d,alpha,Targ,proc):
    sol=IK_irb4600(d,a,alpha,Targ,proc)
    
    check=-1
    for k in range(2):
        arr_sum=np.sum(sol[:,k])
        array_has_nan= np.isnan(arr_sum)
        if array_has_nan == True:
            check=0
            
        else:
            check=1
            break
    return check    
####Find best target for drilling
#This minimize the joint configuration of the first point of the traj and assign to the rest of traj points same orientation
#def OneDOFalg(PointList,tool,wobj,a,d,alpha,d_theta,Llim,Ulim):
    poss_rot=int(2*math.pi/d_theta)
    All_sol_rot=np.zeros((poss_rot,6))
    Cost=np.ones(poss_rot)*10000
    
    fin_rot=0
    fin_rot_small=0
    small_theta=d_theta/4
    small_rot=int(math.pi/(2*small_theta))
    All_small_rot=np.zeros((small_rot,6))
    Cost_small=np.ones(small_rot)*10000
    for i in range(len(PointList)):

        
       
        if i>0 and PointList[i,0]==PointList[i-1,0]:
            Temp_targ=np.zeros(7)
            Temp_targ[0:3]=PointList[i,1:4]
            Temp_targ[3:7]=RotQuat_arZ(PointList[i,4:8],d_theta*fin_rot)
            L6=FromTCPtoLink6(PointList[i,1:8],tool)
            L6[0]=(L6[0]+wobj[0])*0.001
            L6[1]=(L6[1]+wobj[1])*0.001
            L6[2]=(L6[2]+wobj[2])*0.001
            reach=Reach_check(a,d,alpha, L6)
            if reach==1:
                PointList[i,-1]=1
                PointList[i,4:8]=Temp_targ[3:7]
            else:
                PointList[i,-1]=0
                #for rr in range(small_rot):
                #
                #    Temp_targ[3:7]=RotQuat_arZ(PointList[i,4:8],small_theta*rr)
                #    L6=FromTCPtoLink6(Temp_targ,tool)
                #    L6[0]=(L6[0]+wobj[0])*0.001
                #    L6[1]=(L6[1]+wobj[1])*0.001
                #    L6[2]=(L6[2]+wobj[2])*0.001
                #    sol=IK_irb4600(d,a,alpha,L6)
                #    min_join=choose_min_joints(sol,Llim,Ulim)
                #    if isinstance(min_join, str)==True:
                #        All_small_rot[rr,:]=10000
                #    else:
                #        All_small_rot[rr,:]=np.squeeze(min_join)
                #        Cost_small[rr]= abs(All_small_rot[rr,0])/(Ulim[0]-Llim[0])+abs(All_small_rot[rr,1]-math.radians(30))/(Ulim[1]-Llim[1])+abs(All_small_rot[rr,2]-math.radians(-105/2))/(Ulim[2]-Llim[2])+abs(All_small_rot[rr,3])/(Ulim[3]-Llim[3])+abs(All_small_rot[rr,4]-math.radians(-2.5))/(Ulim[4]-Llim[4])+abs(All_small_rot[rr,5])/(Ulim[5]-Llim[5])   
                #minCost_small=np.nanmin(np.nanmin(Cost_small))
                #print("cost : "+str(Cost_small))
                #print("mincost : "+str(minCost_small))
                #if minCost_small==10000:
                #    PointList[i,-1]=0
                #else:
                #    PointList[i,-1]=1
                #    final_rot_small=np.argwhere(Cost_small==minCost_small)
                #    print("argwhere fin rot:"+str(final_rot_small))
                #    if final_rot_small.shape[0] > 1:
                #        fin_rot_small=int(final_rot_small[0])
                #    else:
                #        fin_rot_small=int(final_rot_small)    
                #    print("fin_rot"+str(fin_rot_small))
                #    PointList[i,4:8]=RotQuat_arZ(PointList[i,4:8],small_theta*fin_rot_small)    

        else:
            for r in range(poss_rot):
                Temp_targ=np.zeros(7)
                Temp_targ[0:3]=PointList[i,1:4]
                Temp_targ[3:7]=RotQuat_arZ(PointList[i,4:8],d_theta*r)
                L6=FromTCPtoLink6(Temp_targ,tool)
                L6[0]=(L6[0]+wobj[0])*0.001
                L6[1]=(L6[1]+wobj[1])*0.001
                L6[2]=(L6[2]+wobj[2])*0.001
                sol=IK_irb4600(d,a,alpha,L6)
                min_join=choose_min_joints(sol,Llim,Ulim)
                if isinstance(min_join, str)==True:
                    All_sol_rot[r,:]=10000
                else:
                    All_sol_rot[r,:]=np.squeeze(min_join)
                    Cost[r]= abs(All_sol_rot[r,0])/(Ulim[0]-Llim[0])+abs(All_sol_rot[r,1]-math.radians(30))/(Ulim[1]-Llim[1])+abs(All_sol_rot[r,2]-math.radians(-105/2))/(Ulim[2]-Llim[2])+abs(All_sol_rot[r,3])/(Ulim[3]-Llim[3])+abs(All_sol_rot[r,4]-math.radians(-2.5))/(Ulim[4]-Llim[4])+abs(All_sol_rot[r,5])/(Ulim[5]-Llim[5])   
            minCost=np.nanmin(np.nanmin(Cost))
            if minCost==10000:
                PointList[i,-1]=0
            else:
                PointList[i,-1]=1
                fin_rot=np.argwhere(Cost==minCost)
                if fin_rot.shape[0] > 1:
                    fin_rot=int(fin_rot[0])
                else:
                    fin_rot=int(fin_rot)  
                PointList[i,4:8]=RotQuat_arZ(PointList[i,4:8],d_theta*fin_rot)
    return PointList                


#This assign to the traj targets the orientation that gives best number of reachable points
def MaxReach1DoF(PointList,LabelSet,tool,wobj,a,d,alpha,d_theta,safe_out,case,hole_appr):
    poss_rot=int(2*math.pi/d_theta)
    
    for i in LabelSet:
        lab=np.argwhere(PointList[:,0]==i)
      
        sumReach=np.zeros((poss_rot,len(lab)))
        for j in range(len(lab)):
            for r in range(poss_rot):
               
                Temp_targ=np.zeros(8)
                if case==1:
                    Temp_targ[0:8]=AlongZ(np.reshape(PointList[lab[j],0:8],8),hole_appr)
                else:
                    Temp_targ[1:4]=PointList[lab[j],1:4]    
                
                Temp_targ[4:8]=RotQuat_arZ(PointList[int(lab[j]),4:8],d_theta*r)
                L6=FromTCPtoLink6(Temp_targ[1:8],tool)
                L6[0]=(L6[0]+wobj[0])*0.001
                L6[1]=(L6[1]+wobj[1])*0.001
                L6[2]=(L6[2]+wobj[2])*0.001
                reach_tar=Reach_check_red(a,d,alpha, L6,case)
                Temp_targ=np.zeros(8)
                if case==1:
                    Temp_targ[0:8]=AlongZ(np.reshape(PointList[lab[j],0:8],8),safe_out)
                    xj5=Temp_targ[1]
                    yj5=Temp_targ[2]
                    zj5=Temp_targ[3]
                    Ikred=IK_irb4600_Red(d, a, alpha, [(xj5+wobj[0])*0.001,(yj5+wobj[1])*0.001,(zj5+wobj[2])*0.001],1,case)
                    arrsum=np.sum(Ikred)
                    arrNan=np.isnan(arrsum)
                    if arrNan ==True:
                        reach_app=0
                    else:
                        reach_app=1    
                    #Temp_targ[0:8]=AlongZ(np.reshape(PointList[lab[j],0:8],8), safe_out)
                else:
                    Temp_targ[1:4]=PointList[lab[j],1:4]
                
               
                    Temp_targ[4:8]=RotQuat_arZ(PointList[int(lab[j]),4:8],d_theta*r)
                    L6=FromTCPtoLink6(Temp_targ[1:8],tool)
                    L6[0]=(L6[0]+wobj[0])*0.001
                    L6[1]=(L6[1]+wobj[1])*0.001
                    L6[2]=(L6[2]+wobj[2])*0.001
                    
                    reach_app=Reach_check_red(a,d,alpha, L6, case)
                sumReach[r,j]=reach_app+reach_tar
        argmaxi=sumReach.sum(axis=1)
        final_rot=argmaxi.argmax()
        for k in range(len(lab)):
            if sumReach[final_rot,k]>1:
                PointList[int(lab[k]),4:8]=RotQuat_arZ(PointList[int(lab[k]),4:8],d_theta*final_rot)
                PointList[int(lab[k]),-1]=1
            else:
                PointList[int(lab[k]),-1]=0
    return PointList                






###Function to choose the closest next joint pose
###Cost=sum(abs(j_fin-j_in)/range)
def choose_min_joints(poss_j,l_lim_j,u_lim_j):
    len_sol=len(poss_j[0,:])
    num_sol=np.zeros(len_sol)
    mid_j=np.zeros(6)
    for k in range (len_sol):
        sum=np.sum(poss_j[:,k])
        num_sol[k]=np.isnan(sum)
    for y in range(6):
        mid_j[y]=(u_lim_j[y]+l_lim_j[y])/2
    if len(np.argwhere(num_sol==1))==8:
        return("Next Joint pose doesn't exist")
    else:
        joint_sol=np.zeros(6)
        av_j_pose=np.argwhere(num_sol==0)
        Cost_f=np.zeros(len(av_j_pose))
        for i in range (len(av_j_pose)):
            Cost_f[i]= abs(poss_j[0,av_j_pose[i]]-mid_j[0])/(u_lim_j[0]-l_lim_j[0])+abs(poss_j[1,av_j_pose[i]]-mid_j[1])/(u_lim_j[1]-l_lim_j[1])+abs(poss_j[2,av_j_pose[i]]-mid_j[2])/(u_lim_j[2]-l_lim_j[2])+abs(poss_j[3,av_j_pose[i]]-mid_j[3])/(u_lim_j[3]-l_lim_j[3])+abs(poss_j[4,av_j_pose[i]]-mid_j[4])/(u_lim_j[4]-l_lim_j[4])+abs(poss_j[5,av_j_pose[i]]-mid_j[5])/(u_lim_j[5]-l_lim_j[5])
        min_Cost=np.min(np.min(Cost_f))
        ind_sol=np.argwhere(Cost_f==min_Cost)
        joint_sol= poss_j[:,av_j_pose[ind_sol]]
        return joint_sol

def choose_min_Lastjoints(poss_j,l_lim_j,u_lim_j):
    len_sol=len(poss_j[0,:])
    num_sol=np.zeros(len_sol)
    mid_j=np.zeros(6)
    for k in range (len_sol):
        sum=np.sum(poss_j[:,k])
        num_sol[k]=np.isnan(sum)
    for y in range(6):
        mid_j[y]=(u_lim_j[y]+l_lim_j[y])/2
    if len(np.argwhere(num_sol==1))==len_sol:
        return("Next Joint pose doesn't exist")
    else:
        joint_sol=np.zeros(6)
        av_j_pose=np.argwhere(num_sol==0)
        Cost_f=np.zeros(len(av_j_pose))
        for i in range (len(av_j_pose)):
            Cost_f[i]= abs(poss_j[4,av_j_pose[i]]-mid_j[4])/(u_lim_j[4]-l_lim_j[4])+abs(poss_j[5,av_j_pose[i]]-mid_j[5])/(u_lim_j[5]-l_lim_j[5])
        min_Cost=np.min(np.min(Cost_f))
        ind_sol=np.argwhere(Cost_f==min_Cost)
        joint_sol= poss_j[:,av_j_pose[ind_sol]]
        return joint_sol



###Function to choose the closest next joint pose
###Cost=sum(abs(j_fin-j_in)/range)
def choose_next_joints(actual_j,possible_j,l_lim_j,u_lim_j,weight_finalj):
    w=weight_finalj
    len_sol=len(possible_j[0,:])
    num_sol=np.zeros(len_sol)
    for k in range (len_sol):
        sum=np.sum(possible_j[:,k])
        num_sol[k]=np.isnan(sum)
    
    if len(np.argwhere(num_sol==1))==len_sol:
        return("Next Joint pose doesn't exist")
    else:
        joint_sol=np.zeros(6)
        av_j_pose=np.argwhere(num_sol==0)
        Cost_f=np.zeros(len(av_j_pose))
        
        for i in range (len(av_j_pose)):
            Cost_f[i]= abs(possible_j[0,av_j_pose[i]]-actual_j[0])/(u_lim_j[0]-l_lim_j[0])+abs(possible_j[1,av_j_pose[i]]-actual_j[1])/(u_lim_j[1]-l_lim_j[1])+abs(possible_j[2,av_j_pose[i]]-actual_j[2])/(u_lim_j[2]-l_lim_j[2])+abs(possible_j[3,av_j_pose[i]]-actual_j[3])/(u_lim_j[3]-l_lim_j[3])+abs(possible_j[4,av_j_pose[i]]-actual_j[4])/(u_lim_j[4]-l_lim_j[4])+w*abs(possible_j[5,av_j_pose[i]]-actual_j[5])/(u_lim_j[5]-l_lim_j[5])
        min_Cost=np.min(np.min(Cost_f))
        ind_sol=np.argwhere(Cost_f==min_Cost)
        joint_sol= possible_j[:,av_j_pose[ind_sol]]
        if joint_sol[3]+2*math.pi<u_lim_j[3]:
            if abs(actual_j[3]-(joint_sol[3]+2*math.pi))<abs(actual_j[3]-joint_sol[3]):
                joint_sol[3]=joint_sol[3]+2*math.pi
        if joint_sol[3]-2*math.pi>l_lim_j[3]:
            if abs(actual_j[3]-(joint_sol[3]-2*math.pi))<abs(actual_j[3]-joint_sol[3]):
                joint_sol[3]=joint_sol[3]-2*math.pi    
        
        if joint_sol[5]+2*math.pi<u_lim_j[5]:
            if abs(actual_j[5]-(joint_sol[5]+2*math.pi))<abs(actual_j[5]-joint_sol[5]):
                joint_sol[5]=joint_sol[5]+2*math.pi
        if joint_sol[5]-2*math.pi>l_lim_j[5]:
            if abs(actual_j[5]-(joint_sol[5]-2*math.pi))<abs(actual_j[5]-joint_sol[5]):
                joint_sol[5]=joint_sol[5]-2*math.pi        
        return joint_sol

###"Obstacle Avoidance"-> move robot in joints space moving j5 along safety surface
def Move_safely(actual_j,final_j,popt,v_lin,w_lin, egm_step,Tool, wobj):
    interp_in=[]
    interp_final=[]
    proc=1
    
    T05in=Dir_kin_irb4600(a,d,alpha, actual_j,5,Tool)
    x_act=T05in[0,3]*1000
    y_act=T05in[1,3]*1000
    z_act=paraBolEqn([x_act-wobj[0],y_act-wobj[1]],popt[0],popt[1],popt[2],popt[3],popt[4])
    jsafein=np.zeros(6)
    jsafein[0:3]=IK_irb4600_Red(d,a,alpha,[(x_act)/1000,(y_act)/1000,(z_act+wobj[2])/1000], 1,proc)
    jsafein[3]=actual_j[3]
    jsafein[4]=actual_j[4]
    jsafein[5]=actual_j[5]

    inter_tosafe=j_int(actual_j,jsafein,w_lin,egm_step) ###First move only j123
    inter_tosafe=np.hstack((inter_tosafe, np.tile(inter_tosafe[:,[-1]], 50)))

    
    T05fin=Dir_kin_irb4600(a,d,alpha,final_j,5,Tool)
    x_fin= T05fin[0,3]*1000
    y_fin= T05fin[1,3]*1000
    z_fin=paraBolEqn([x_fin-wobj[0],y_fin-wobj[1]],popt[0],popt[1],popt[2],popt[3],popt[4])
    time_traj=Const_vel(x_act,y_act,z_act,x_fin,y_fin,z_fin, v_lin)
    x_arr=interpolate(x_act,x_fin,time_traj,egm_step)
    y_arr=interpolate(y_act,y_fin,time_traj,egm_step)
    z_arr=np.zeros(x_arr.shape[0])
    inter_surf=np.zeros((6,z_arr.shape[0]+10))
    for k in range(z_arr.shape[0]+10):
        if k<z_arr.shape[0]:
            z_arr[k]=paraBolEqn([x_arr[k]-wobj[0],y_arr[k]-wobj[1]],popt[0],popt[1],popt[2],popt[3],popt[4])
            inter_surf[0:3,k]=IK_irb4600_Red(d,a,alpha,[x_arr[k]/1000,y_arr[k]/1000,(z_arr[k]+wobj[2])/1000], 1,proc)
        else:
            inter_surf[0:3,k]=inter_surf[0:3,z_arr.shape[0]-1]

    inter_surf[3,:]=np.linspace(actual_j[3],final_j[3], z_arr.shape[0]+10) 
    inter_surf[4,:]=np.linspace(actual_j[4],final_j[4], z_arr.shape[0]+10)
    inter_surf[5,:]=np.linspace(actual_j[5],final_j[5], z_arr.shape[0]+10) 

    interp_in=np.append(inter_tosafe,inter_surf, axis=1)

    prefinal_j=[interp_in[0,-1],interp_in[1,-1],interp_in[2,-1],interp_in[3,-1],interp_in[4,-1],interp_in[5,-1]]      
    
    inter_toTarg=j_int(prefinal_j,final_j,w_lin,egm_step)

    interp_final=np.append(interp_in,inter_toTarg, axis=1)

    
    ###to implement
    return interp_final
################################################################################


####Test############
#sol=np.zeros((6,8))
#sol=IK_irb4600(d,a,alpha,[1.0882457091226201, 0.63598500428926841, 0.30712512407336851, 0.6152898413946069, 0.5394115028996764, 0.21279645332290476, -0.5340143360147782]) 
#siu=len(sol)
#reach=Reach_check(a,d,alpha,[0.92072,0.55355,0.36677,0.70335,-0.40233,0.58260,0.06326]) 
##print(sol[:,1])
#sol_deg=np.zeros((8,6))
#for k in range(8):
#   for t in range(6):
#     sol_deg[k,t]=math.degrees(sol[t,k])
#print("starting pose")
#print(sol_deg)         
#
#T=Dir_kin_irb4600(a,d,alpha,sol[:,0],6)
#print("dir_kin :"+str(T[0:3,3]))
#print("error:"+str([0.92072-T[0,3],0.55355-T[1,3],0.36677-T[2,3]]))
#for h in range(8):
#    check=sol[5,h]
#    if math.isnan(check)!= True:
#       Jacobian=get_Jac(sol[:,h],d,a,alpha)
#       print("Jacobian")
#       print(Jacobian)              
#       jacobian_T=Jacobian.transpose()
#       man = np.matmul(Jacobian, jacobian_T)
#       print("manipulability" + str(h+1))
#       manipulability = math.sqrt(np.linalg.det(man))
#       print(manipulability)
#       jac_w=Jacobian[0:3,:]
#       jac_v=Jacobian[3:,:]
#       jac_vT=jac_v.transpose()
#       jac_wT=jac_w.transpose()
#
#       man_v = np.matmul(jac_v, jac_vT)
#       man_w = np.matmul(jac_w, jac_wT)
#       
#       w_v=math.sqrt(np.linalg.det(man_v))
#       print("manip_lin :"+str(w_v))
#       w_w=math.sqrt(np.linalg.det(man_w))
#       print("manip_ang :"+str(w_w))
#    else:
#        print("No solution #"+str(h+1))   #
#sol2=np.zeros((6,8))
#sol2=IK_irb4600(d,a,alpha,[0.92072,0.55355,0.61677,0.70335,-0.40233,0.58260,0.06326]) 
##print(sol) 
##print(sol[:,1])
#sol_deg2=np.zeros((8,6))
#for k in range(8):
#   for t in range(6):
#     sol_deg2[k,t]=math.degrees(sol2[t,k])
#print("end pose")
#print(sol_deg2)         
#start_joint2=np.zeros((4,6))
#
#start_joint=choose_min_joints(sol,min_angle,max_angle)
#start_joint2[2,:]=np.squeeze(start_joint)
#
#print("start joint: "+str(np.degrees(start_joint)))
#
#next_joint=choose_next_joints(start_joint,sol2,min_angle,max_angle)
#next_joint_d=np.degrees(next_joint[:])
#print("next joint: "+str(next_joint_d))
#interp0=[]
#
##print(np.linspace(0.50,0.177,9))
#interp=j_int(start_joint,next_joint,math.radians(10),0.004)
#
#print("interp :"+str(np.degrees(interp)))
#
#
#sol3=IK_irb4600(d,a,alpha,[0.92072,0.0,0.61677,0.70335,-0.40233,0.58260,0.06326]) 
##print(sol) 
##print(sol[:,1])
#sol_deg3=np.zeros((8,6))
#for k in range(8):
#   for t in range(6):
#     sol_deg3[k,t]=math.degrees(sol3[t,k])
#print("end pose")
#print(sol_deg2)         
#
#
##start_joint=choose_min_joints(sol,min_angle,max_angle)
##print("start joint: "+str(np.degrees(start_joint)))
#
#next_joint2=choose_next_joints(next_joint,sol3,min_angle,max_angle)
#next_joint_d=np.degrees(next_joint2[:])
#print(next_joint_d[0][0][0])
#
#
##print(np.linspace(0.50,0.177,9))
#interp2=j_int(next_joint,next_joint2,math.radians(2),1)
#
#print("interp2 :"+str(np.degrees(interp2)))
#
#
#final_interp=np.append(interp,interp2,axis=1)
#print("final interp :"+str(np.degrees(final_interp)))
#
#print(final_interp.shape[1])
#
#twoJoints=np.append(next_joint,next_joint2,axis=1)
#trejoints=np.append(twoJoints,start_joint,axis=1)
#print(next_joint[5][0][0])
#print(next_joint2)
#print(trejoints)
####################################
#Targ_fori=np.array(np.loadtxt('/home/gabriele/catkin_ws/src/frank/Data/punti_e_quat4.txt', usecols=range(7)))
#dbs=DBSCAN(eps=30, min_samples=3).fit(Targ_fori[:,0:2])     #For LMA_workpiece eps=30 samp=3   Our piece eps=70,s=3
#labels=dbs.labels_
#
#NTraj= len(set(labels)) - (1 if -1 in labels else 0) #number of trajectories
##Extremes=np.zeros((NTraj*2,4)) #inizialize extremes vector
##Ordered_tag_fori=np.ones((len(Targ_fori),7))*(-1) # label/x/y/z/ex/ey/ez (forse aggiungere alzata robot tra traiettorie ultimo punto +off_z)
#Ordered_tag_fori=np.ones((len(Targ_fori),9))*(-1)
##
##
###########!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#Ordered_tag_fori=Order_from_clust(Targ_fori, Ordered_tag_fori, labels, NTraj)
#egm_time_step=0.004
#mmtom=0.001
#DegTorad= math.pi/180
####Joint limits####
#min_angle=[math.radians(-180),math.radians(-90),math.radians(-180),math.radians(-400),math.radians(-125),math.radians(-400)]
#max_angle=[math.radians(180),math.radians(150),math.radians(75),math.radians(400),math.radians(120),math.radians(400)]
#safeOut=100
#dTheta=2*math.pi/8
#wobj=[-2124,0,-6592]
#tool=[378,0,281.88,0.707106781,0,0.707106781,0]
#Ordered_tag_fori=MaxReach1DoF(Ordered_tag_fori,NTraj,tool,wobj,a,d,alpha,dTheta,safeOut,1,10)
#print("Orde_fori+reach :" +str(Ordered_tag_fori[:,-1]))
#print("labels :"+str(Ordered_tag_fori[:,0]))
#
###pubnum.publish(0)
#####       
#MIN_angle=[math.radians(-180),math.radians(-90),math.radians(-180),math.radians(-400),math.radians(-125),math.radians(-400)]
#MAX_angle=[math.radians(180),math.radians(150),math.radians(75),math.radians(400),math.radians(120),math.radians(400)]
#
#####!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#Ordered_tag_fori_clear=Remove_no_reach(Ordered_tag_fori, 8) ####8 is the column with reach check
#q1_2_3=np.zeros((len(Ordered_tag_fori),3))
#q1_2_3_safe=np.zeros((len(Ordered_tag_fori),3))
#q1_2_3_coll=np.zeros((len(Ordered_tag_fori),3))
#for i in range(len(Ordered_tag_fori)):
#    
#    tcp_in2=[Ordered_tag_fori[i,1],Ordered_tag_fori[i,2],Ordered_tag_fori[i,3],Ordered_tag_fori[i,4],Ordered_tag_fori[i,5],Ordered_tag_fori[i,6],Ordered_tag_fori[i,7]]
#    L6_in2=FromTCPtoLink6(tcp_in2,tool)
#    L6_in2[0]=(L6_in2[0]+wobj[0])*mmtom
#    L6_in2[1]=(L6_in2[1]+wobj[1])*mmtom
#    L6_in2[2]=(L6_in2[2]+wobj[2])*mmtom
#    sol_in2=IK_irb4600(d,a,alpha,L6_in2)
#    joints_in2=choose_min_joints(sol_in2,MIN_angle,MAX_angle)
#    q1_2_3[i,0:3]=np.degrees(np.squeeze(joints_in2[0:3]))
#
#    tcp_in2_safe=[Ordered_tag_fori[i,1],Ordered_tag_fori[i,2],Ordered_tag_fori[i,3]+100,Ordered_tag_fori[i,4],Ordered_tag_fori[i,5],Ordered_tag_fori[i,6],Ordered_tag_fori[i,7]]
#    L6_in2_safe=FromTCPtoLink6(tcp_in2_safe,tool)
#    L6_in2_safe[0]=(L6_in2_safe[0]+wobj[0])*mmtom
#    L6_in2_safe[1]=(L6_in2_safe[1]+wobj[1])*mmtom
#    L6_in2_safe[2]=(L6_in2_safe[2]+wobj[2])*mmtom
#    sol_in2_safe=IK_irb4600(d,a,alpha,L6_in2_safe)
#    joints_in2_safe=choose_min_joints(sol_in2_safe,MIN_angle,MAX_angle)
#    q1_2_3_safe[i,0:3]=np.degrees(np.squeeze(joints_in2_safe[0:3]))
#
#    tcp_in2_coll=[Ordered_tag_fori[i,1],Ordered_tag_fori[i,2],Ordered_tag_fori[i,3]-100,Ordered_tag_fori[i,4],Ordered_tag_fori[i,5],Ordered_tag_fori[i,6],Ordered_tag_fori[i,7]]
#    L6_in2_coll=FromTCPtoLink6(tcp_in2_coll,tool)
#    L6_in2_coll[0]=(L6_in2_coll[0]+wobj[0])*mmtom
#    L6_in2_coll[1]=(L6_in2_coll[1]+wobj[1])*mmtom
#    L6_in2_coll[2]=(L6_in2_coll[2]+wobj[2])*mmtom
#    sol_in2_coll=IK_irb4600(d,a,alpha,L6_in2_coll)
#    joints_in2_coll=choose_min_joints(sol_in2_coll,MIN_angle,MAX_angle)
#    q1_2_3_coll[i,0:3]=np.degrees(np.squeeze(joints_in2_coll[0:3]))
#
#fig=plt.figure()
#ax = plt.axes(projection="3d")
#joints_in2_coll[4][0][0][0]=joints_in2_coll[4][0][0][0]+math.radians(20)
#ax.scatter3D(q1_2_3[:,0],q1_2_3[:,1],q1_2_3[:,2], c='b')
#ax.scatter3D(q1_2_3_safe[:,0],q1_2_3_safe[:,1],q1_2_3_safe[:,2], c='g')
#ax.scatter3D(q1_2_3_coll[:,0],q1_2_3_coll[:,1],q1_2_3_coll[:,2], c='r')
#ax.set_xlabel('J1')
#ax.set_ylabel('J2')
#ax.set_zlabel('J3')
#
#
#fig2=plt.figure()
#ax1=fig2.add_subplot(111)
#ax1.scatter(q1_2_3[:,0],q1_2_3[:,1],c='b')
#ax1.scatter(q1_2_3_coll[:,0],q1_2_3_coll[:,1],c='r')
#ax1.scatter(q1_2_3_safe[:,0],q1_2_3_safe[:,1],c='g')
#ax1.set_xlabel('J1')
#ax1.set_ylabel('J2')
#
#
#
##x1.set_xlabel('J1')
##x1.set_ylabel('J2')
#
#fig3=plt.figure()
#ax2=fig3.add_subplot(111)
#ax2.scatter(q1_2_3[:,1],q1_2_3[:,2],c='b')
#ax2.scatter(q1_2_3_coll[:,1],q1_2_3_coll[:,2],c='r')
#ax2.scatter(q1_2_3_safe[:,1],q1_2_3_safe[:,2],c='g')
#ax2.set_xlabel('J2')
#ax2.set_ylabel('J3')
#
#fig4=plt.figure()
#ax3=fig4.add_subplot(111)
#ax3.scatter(q1_2_3[:,0],q1_2_3[:,2],c='b')
#ax3.scatter(q1_2_3_coll[:,0],q1_2_3_coll[:,2],c='r')
#ax3.scatter(q1_2_3_safe[:,0],q1_2_3_safe[:,2],c='g')
#ax3.set_xlabel('J1')
#ax3.set_ylabel('J3')
#
#
#plt.show()

##############################Try to fit 3d curve
#from mpl_toolkits.mplot3d import Axes3D
#import matplotlib
#from matplotlib import cm
#import matplotlib.pyplot as plt
#def SurfacePlot(func, data, fittedParameters):
#    f = plt.figure()
#
#    matplotlib.pyplot.grid(True)
#    axes = Axes3D(f)
#
#    x_data = data[:,0]
#    y_data = data[:,1]
#    z_data = data[:,2]
#
#    xModel = np.linspace(min(x_data), max(x_data), 20)
#    yModel = np.linspace(min(y_data), max(y_data), 20)
#    X, Y = np.meshgrid(xModel, yModel)
#
#    Z = func(np.array([X, Y]), *fittedParameters)
#
#    axes.plot_surface(X, Y, Z, rstride=1, cstride=1, cmap=cm.coolwarm, linewidth=1, antialiased=True)
#
#    axes.scatter(x_data, y_data, z_data) # show data along with plotted surface
#
#    axes.set_title('Surface Plot (click-drag with mouse)') # add a title for surface plot
#    axes.set_xlabel('X Data') # X axis data label
#    axes.set_ylabel('Y Data') # Y axis data label
#    axes.set_zlabel('Z Data') # Z axis data label
#
#    plt.show()
#    plt.close('all') # clean up after using pyplot or else thaere can be memory and process problems
#def ScatterPlot(data):
#    f = plt.figure()
#
#    matplotlib.pyplot.grid(True)
#    axes = Axes3D(f)
#    x_data = data[:,0]
#    y_data = data[:,1]
#    z_data = data[:,2]
#
#    axes.scatter(x_data, y_data, z_data)
#
#    axes.set_title('Scatter Plot (click-drag with mouse)')
#    axes.set_xlabel('X Data')
#    axes.set_ylabel('Y Data')
#    axes.set_zlabel('Z Data')
#
#    plt.show()
#    plt.close('all') # clean up after using pyplot or else thaere can be memory and process problems
#
#
#import scipy as sp
#from scipy.optimize import curve_fit
#
#def paraBolEqn(data,a,b,c,d,e):
#    x,y = data
#    z= -(((x-b)/a)**2+((y-d)/c)**2)+e
#    #z=a + b * x**0 * y**0 + c * x**0 * y**1 + d * x**0 * y**2 
#    #+ e * x**1 * y**0 + f * x**1 * y**1 + g * x**1 * y**2
#    #+ h * x**2 * y**0 + i * x**2 * y**1 + j * x**2 * y**2
#    return z
#Max_dist=472+ 135 #135 il length of J5
#
#for i in range(len(Targ_fori)):
#    Targ_fori[i,:]=AlongZ(Targ_fori[i,:],-Max_dist)
#
#
#popt,pcov=curve_fit(paraBolEqn,np.vstack((Targ_fori[:,0],Targ_fori[:,1])),Targ_fori[:,2],p0=[1,1,1,1,1], maxfev=1000000)
#
#
#ScatterPlot(Targ_fori[:,0:3])
#SurfacePlot(paraBolEqn, Targ_fori[:,0:3], popt)
#z=paraBolEqn([1,1],popt[0],popt[1],popt[2],popt[3],popt[4])
#print(z)