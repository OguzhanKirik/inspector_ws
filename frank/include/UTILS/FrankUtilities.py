from UTILS import FrankCommons
import math
import numpy as np

#ToolR0Matrix = [[0, 0, 1], 
#                [0, 1, 0], 
#               [-1, 0, 0]] #Rotation Matrix that defines the position of the tool with respect to the absolute one
UseXasVersor2 = True #Used in Target2PositionRotMatrix (One Versor (TargetNormal) to define the coordinate system). During coordinate system computation when TargetNormal is +Z, uses +X as second versor. Otherwise it uses the radius between TargetPoint and [0, 0, 0].

def Quaternion2Euler(input):
    "Restituisce gli angoli di eulero partendo dal quanternione. Ordine: alfa (jaw, EZ), Beta (pitch, EY), Gamma (roll, EX)"
    R = Quaternion2RotMatrix(input)
    if (R == -1):
        print("Quaternion2Euler, input error")
        return -1
    return RotMatrix2Euler(R)

def RotMatrix2Euler(input):
    "Restitusce gli angoli di eulero partendo dalla matrice di rotazione. Ordine: alfa (jaw, EZ), Beta (pitch, EY), Gamma (roll, EX)"
    res = FrankCommons.Check_Matrix(input)
    if res == -1:
        print("RotMatrix2Euler, input error")
        return -1
    [n, m] = res
    if n != 3 or m != 3:
        print("RotMatrix2Euler, input error")
        return -1

    sy = math.sqrt(input[0][0]**2 + input[1][0]**2)
    singular = sy < FrankCommons.tol
    output = [0,0,0]
    output[1] = math.atan2(-input[2][0],sy)
    if singular:
        output[0] = 0
        output[2] = math.atan2(-input[1][2],input[1][1])
    else:
        output[0] = math.atan2(input[1][0],input[0][0])
        output[2] = math.atan2(input[2][1],input[2][2])

    return output

def Quaternion2RotMatrix(input):
    "Restituisce la matrice di rotazione partendo dal quaternione"
    n = FrankCommons.Check_Array(input)
    if(n != 4):
        print("Quaternion2RotMatrix, input error")
        return -1
    qw=input[0]
    qx=input[1]
    qy=input[2]
    qz=input[3]
    output = [[0,0,0],[0,0,0],[0,0,0]]
    output[0][0] = qw**2+qx**2-qy**2-qz**2
    output[0][1] = 2*qx*qy-2*qw*qz
    output[0][2] = 2*qx*qz+2*qw*qy
    output[1][0] = 2*qx*qy+2*qw*qz
    output[1][1] = qw**2-qx**2+qy**2-qz**2
    output[1][2] = 2*qy*qz-2*qw*qx
    output[2][0] = 2*qx*qz-2*qw*qy
    output[2][1] = 2*qy*qz+2*qw*qx
    output[2][2] = qw**2-qx**2-qy**2+qz**2
    return output

def Euler2RotMatrix(input):
    "Costruisce la matrice di rotazione partendo dagli angoli di Eulero. Ordine: alfa (jaw, EZ), Beta (pitch, EY), Gamma (roll, EX)"
    res = FrankCommons.Check_Array(input)
    if(res != 3):
        print("Euler2RotMatrix, input error")
        return -1
    alfa=input[0]
    beta=input[1]
    gamma=input[2]
    output = [[0,0,0],[0,0,0],[0,0,0]]
    output[0][0] = round(math.cos(alfa)*math.cos(beta),10)
    output[0][1] = round(math.cos(alfa)*math.sin(beta)*math.sin(gamma)-math.sin(alfa)*math.cos(gamma),10)
    output[0][2] = round(math.cos(alfa)*math.sin(beta)*math.cos(gamma)+math.sin(alfa)*math.sin(gamma),10)
    output[1][0] = round(math.sin(alfa)*math.cos(beta),10)
    output[1][1] = round(math.sin(alfa)*math.sin(beta)*math.sin(gamma)+math.cos(alfa)*math.cos(gamma),10)
    output[1][2] = round(math.sin(alfa)*math.sin(beta)*math.cos(gamma)-math.cos(alfa)*math.sin(gamma),10)
    output[2][0] = round(-math.sin(beta),10)
    output[2][1] = round(math.cos(beta)*math.sin(gamma),10)
    output[2][2] = round(math.cos(beta)*math.cos(gamma),10)
    return output
def Euler2RotMatrix_deg(input):
    "Costruisce la matrice di rotazione partendo dagli angoli di Eulero. Ordine: alfa (jaw, EZ), Beta (pitch, EY), Gamma (roll, EX)"
   
    res = FrankCommons.Check_Array(input)
    if(res != 3):
        print("Euler2RotMatrix, input error")
        return -1
    alfa=math.radians(input[0])
    beta=math.radians(input[1])
    gamma=math.radians(input[2])
    output = [[0,0,0],[0,0,0],[0,0,0]]
    output[0][0] = round(math.cos(alfa)*math.cos(beta),10)
    output[0][1] = round(math.cos(alfa)*math.sin(beta)*math.sin(gamma)-math.sin(alfa)*math.cos(gamma),10)
    output[0][2] = round(math.cos(alfa)*math.sin(beta)*math.cos(gamma)+math.sin(alfa)*math.sin(gamma),10)
    output[1][0] = round(math.sin(alfa)*math.cos(beta),10)
    output[1][1] = round(math.sin(alfa)*math.sin(beta)*math.sin(gamma)+math.cos(alfa)*math.cos(gamma),10)
    output[1][2] = round(math.sin(alfa)*math.sin(beta)*math.cos(gamma)-math.cos(alfa)*math.sin(gamma),10)
    output[2][0] = round(-math.sin(beta),10)
    output[2][1] = round(math.cos(beta)*math.sin(gamma),10)
    output[2][2] = round(math.cos(beta)*math.cos(gamma),10)
    return output
def Euler2Quaternion(input):
    "Costruisce il quaternione partendo dagli angoli di Eulero. Ordine: alfa (jaw, EZ), Beta (pitch, EY), Gamma (roll, EX)"
    res = FrankCommons.Check_Array(input)
    if(res != 3):
        print("Euler2Quaternion, input error")
        return -1
    alfa = input[0]
    beta = input[1]
    gamma = input[2]
    ca = math.cos(alfa/2)
    sa = math.sin(alfa/2)
    cb = math.cos(beta/2)
    sb = math.sin(beta/2)
    cg = math.cos(gamma/2)
    sg = math.sin(gamma/2)
    output = [0, 0, 0, 0]
    output[0] = cg*cb*ca+sg*sb*sa
    output[1] = sg*cb*ca-cg*sb*sa
    output[2] = cg*sb*ca+sg*cb*sa
    output[3] = cg*cb*sa-sg*sb*ca

    return output

def RotMatrix2Quaternion(input):
    "Costruisce il quaternione partendo dalla matrice di rotazione"
    eul = RotMatrix2Euler(input)
    if eul==-1:
        print("RotMatrix2Quaternion, input error")
        return -1
    return Euler2Quaternion(eul)

def Target2PosRT(TargetPoint, TargetNormal, ToolLength):
    "Converte un target (posizione e normale alla superficie) in una posizione del tool e matrice di rotazione"
    err = FrankCommons.Check_Array(TargetPoint)
    if err != 3:
        print("Target2PosRT, TargetPoint error")
        return -1
    err = FrankCommons.Check_Versor(TargetNormal)
    if err!= 3:
        print("Target2PosRT, TargetNormal error")
        return -1

    v1 = [-TargetNormal[0], -TargetNormal[1], -TargetNormal[2]] #Il primo versore è semplicemente la normale alla superficie cambiata di segno

    VerticalVersor = [0, 0, 0]
    if abs(v1[0]) < FrankCommons.tol and abs(v1[1]) < FrankCommons.tol and (1 - abs(v1[2]) < FrankCommons.tol):
        #TargetNormal è diretta come il versore Z
        #Calcolo il versore nel piano XY e diretto verso il TargetPoint
        if UseXasVersor2:
            VerticalVersor = [1, 0, 0]
        else:
            amp = math.sqrt(TargetPoint[0]**2 + TargetPoint[1]**2 + TargetPoint[2]**2)
            v3 = [0, 0, 0]
            VerticalVersor[0] = TargetPoint[0] / amp
            VerticalVersor[1] = TargetPoint[1] / amp
            VerticalVersor[2] = TargetPoint[2] / amp
    else:
        VerticalVersor = __CalcolaVersore(TargetNormal)

    return Target3rdVersor2PosRT(TargetPoint, TargetNormal, VerticalVersor, ToolLength)

def Target3rdVersor2PosRT(TargetPoint, TargetNormal, VerticalVersor, ToolLength):
    '''
    Calculate the Tool position with target point and Normal to the surface. Tool il expected to exits from Z+.

    Input:
    1. Target Point Coordinate [X, Y, Z]
    2. Target Normal (Versor).  Normal to the surface the robot must point. The third versor will point to to the surface (-TargetNormal)
    3. Vertical Versor in robot coordinate system
    4. Length of the tool

    Output:
    1. Tool Coordinate
    2. Tool Rotation Matrix
    '''
    err = FrankCommons.Check_Array(TargetPoint)
    if err != 3:
        print("Target3rdVersor2PosRT, TargetPoint error")
        return -1
    err = FrankCommons.Check_Versor(TargetNormal)
    if err!= 3:
        print("Target3rdVersor2PosRT, TargetNormal error")
        return -1
    err = FrankCommons.Check_Versor(VerticalVersor)
    if err!= 3:
        print("Target3rdVersor2PosRT, VerticalVersor error")
        return -1
    if not (type(ToolLength) is float or type(ToolLength) is int):
        print("Target3rdVersor2PosRT, ToolLength must be float or int")
        return -1

    ToolCoordinate = [TargetPoint[0] + ToolLength*TargetNormal[0], TargetPoint[1] + ToolLength*TargetNormal[1], 
                      TargetPoint[2] + ToolLength*TargetNormal[2]]

    #Bisogna ora calcolare i tre versori della terna come fosse quella del robot (e non del tool).
    #v1 = [-TargetNormal[0], -TargetNormal[1], -TargetNormal[2]] #Il primo versore è semplicemente la normale alla superficie cambiata di segno
    v3 = [-TargetNormal[0], -TargetNormal[1], -TargetNormal[2]] #First Versor is opposite direction of the Target Normal (tool must point Target Object)
    
    v1 = VerticalVersor #Usato v3 perché il calcolo del secondo versore è fatto (nell'altra funzione) considerando la direzione Z assoluta (verticale) come preferibile

    #Calcola il terzo versore come prodotto dei precedenti
    v2 = FrankCommons.crossProduct(v3, v1)

    ToolRT = FrankCommons.VersorsToRotMatrix(v1, v2, v3) #[[v1[0], v2[0], v3[0]], [v1[1], v2[1], v3[1]], [v1[2], v2[2], v3[2]]]
    ToolRT2 = FrankCommons.VersorsToRotMatrix(list(-np.asarray(v1)), list(-np.asarray(v2)), v3)
    #ToolRT = FrankCommons.MatrixProduct(ToolR0Matrix, ToolRT)

    return [ToolCoordinate, ToolRT,ToolRT2]

def __CalcolaVersore(TargetNormal):
    "Calcola un versone normale alla TargetNormal con il coseno direttore Z più grande possibile (non verifica che TN sia Z o -Z)"

    #Per trovare il versore, si utilizzano le seguenti tre equazioni:
    # 1. Equazione del piano definito da TargetNormal e Versore asse Z (equazione a * x + b * y + c * z = 0)
    # 2. Piano perpendicolare alla TargetNormal (equazione TN[0] * x + TN[1] * y + TN[2] * z = 0)
    # 3. Sfera di raggio unitario (deve essere un versore) x**2 + y**2 + z**2 = 1.
    #Si individueranno due soluzioni, si prende quella con z più grande

    err = FrankCommons.Check_Versor(TargetNormal)
    if err!= 3:
        print("__CalcolaVersore, TargetNormal error")
        return -1

    #Costruisco un piano che passa per la normale e per l'asse Z
    #Piano per tre punti: [0, 0, 0], la TargetNormal e l'asse Z [0, 0, 1]
    x1 = 0
    y1 = 0
    z1 = 0
    x2 = TargetNormal[0]
    y2 = TargetNormal[1]
    z2 = TargetNormal[2]
    x3 = 0
    y3 = 0
    z3 = 1

    #Equazioni complete, si possono semplificare visto che x1=y1=z1=0
    #aPlane = (y2-y1)*(z3-z1)-(z2-z1)*(y3-y1)
    #bPlane = -((x2-x1)*(z3-z1)-(x3-x1)*(z2-z1))
    #cPlane = (x2-x1)*(y3-y1)-(x3-x1)*(y2-y1)
    #dPlane = -(aPlane*x1+bPlane*y1+cPlane*z1)
    aa = y2 * z3 - z2 * y3
    bb = -(x2 * z3 - x3 * z2)
    cc = x2 * y3 - x3 * y2

    v3 = [0, 0, 0]
    if (abs(aa) < FrankCommons.tol):
        JJ = (z2 * aa - x2 * cc) / (x2 * bb - y2 * aa)
        II = -(x2 * JJ + z2) / x2
        v3[2] = math.sqrt(1 / (II**2 + JJ**2 + 1)) #è già il valore più grande (positivo)
        v3[1] = JJ * v3[2]
        v3[0] = II * v3[2]
    else:
        JJ = (cc * x2 - aa * z2) / (aa * y2 - bb * x2)
        II = -(bb * JJ + cc) / aa
        v3[2] = math.sqrt(1 / (II**2 + JJ**2 + 1)) #è già il valore più grande (positivo)
        v3[1] = JJ * v3[2]
        v3[0] = II * v3[2]

    return v3