tol = 1e-6 #tolerance

def Check_Matrix(input):
    "Verifica che l'input sia una matrice e restituisce un vettore contenente numero di righe e colonne"
    if not type(input) is list: return -1
    n = len(input)
    m=-1
    for i in range(0,len(input)):
        if not type(input[i]) is list: return -1
        if i == 0:
            m = len(input[i])
        else:
            if len(input[i]) != m: return -1
        pass
    return [n,m]

def Check_Array(input):
    "Verifica se input è un vettore e restituisce la lunghezza"
    if not type(input) is list: return -1
    for i in range(0, len(input)):
        if type(input[i]) is list: return -1
        pass
    return len(input)

def Check_Versor(input):
    "Verifica se input è un versore e restituisce la lunghezza"
    res = Check_Array(input)
    if res != 3:
        print("Check_Versor, input error")
        return -1
    a = input[0]**2 + input [1]**2 + input[2]**2
    if abs (a - 1) > tol:
        print("Check_Versor, input error")
        return -1

    return res

def VersorsToRotMatrix(v1, v2, v3):
    "Returns the Rotation Matrix starting from the standard reference system [1,0,0],[0,1,0],[0,0,1]"
    n = Check_Array(v1)
    if n != 3:
        print("VersorsToRotMatrix, input error")
        return -1
    if SumOfSquares(v1)-1 > tol:
        print("VersorsToRotMatrix, input error")
        return -1
    n = Check_Array(v2)
    if n != 3:
        print("VersorsToRotMatrix, input error")
        return -1
    if SumOfSquares(v2)-1 > tol:
        print("VersorsToRotMatrix, input error")
        return -1
    n = Check_Array(v3)
    if n != 3:
        print("VersorsToRotMatrix, input error")
        return -1
    if SumOfSquares(v3)-1 > tol:
        print("VersorsToRotMatrix, input error")
        return -1

    return [[v1[0], v2[0], v3[0]], [v1[1], v2[1], v3[1]], [v1[2], v2[2], v3[2]]]

def RotMatrixToVersors(input):
    "Returns the three versors starting from the Rotation Matrix in the standard reference system [1,0,0],[0,1,0],[0,0,1]"
    nm = Check_Matrix(input)
    if nm == -1:
        print("RotMatrixToVersors, input error")
        return -1
    n = nm[0]
    m = nm[1]
    if n != 3 or m != 3:
        print("RotMatrixToVersors, input error")
        return -1

    v1 = [input[0][0], input[1][0], input[2][0]]
    v2 = [input[0][1], input[1][1], input[2][1]]
    v3 = [input[0][2], input[1][2], input[2][2]]

    return [v1, v2, v3]

def ChangeCoordsToAbsolute(input, offset, RT):
    '''
    Input parameters:
    1. input is a List of Lists of double containing 3D points coordinates
    2. offset is the offset of the current reference system with the abstolute (or parent) one
    3. RT is the rotation matrix of the current reference system
    '''
    if not type(input) is list:
        print("ChangeCoordsToAbsolute, input error")
        return -1
    n = len(input)
    for i in range(0,n):
        m = Check_Array(input[i])
        if m != 3:
            print("ChangeCoordsToAbsolute, input error")
            return -1

    m = Check_Array(offset)
    if m != 3:
        print("ChangeCoordsToAbsolute, offset error")
        return -1

    nm = Check_Matrix(RT)
    if nm == -1:
        print("ChangeCoordsToAbsolute, RT Matrix error")
        return -1

    if nm[0] != 3 or nm[1] != 3:
        print("ChangeCoordsToAbsolute, RT Matrix error")
        return -1

    res = [0] * n

    for i in range(0,n):
        res[i] = MatrixArrayProduct(RT,input[i])
        res[i][0] += offset[0]
        res[i][1] += offset[1]
        res[i][2] += offset[2]

    return res

def TransposeMatrix(input):
    res = Check_Matrix(input)
    if res == -1:
        print("TransposeMatrix, input error")
        return -1
    [n, m] = res
    res = [0]*m
    for i in range(0,m):
        res[i] = [0]*n

    for i in range(0,n):
        for j in range(0,m):
            res[j][i] = input[i][j]

    return res

def MatrixProduct(m1, m2):
    "Resituisce il prodotto fra due matrici. m1 è una matrice nxa e m2 è una matrice axm"
    
    #Controlli *********************************
    res = Check_Matrix(m1)
    if res == -1:
        print("MatrixProduct, input error")
        return -1
    [n, a] = res
    res = Check_Matrix(m2) 
    if res == -1:
        print("MatrixProduct, input error")
        return -1
    [b, m] = res
    if a != b:
        print("MatrixProduct, input error")
        return -1
    #Controlli *********************************

    result = [0]*n
    for i in range(0,n):
        result[i]=[0]*m
    for i in range(0,n):
        for j in range(0,m):
            for k in range(0,a):
                result[i][j] += m1[i][k] * m2[k][j]
    return result

def MatrixArrayProduct(m1, a1):
    '''
    Performs Matrix * Array product
    '''
    res = Check_Matrix(m1)
    if res == -1:
        print("MatrixArrayProduct, m1 error")
        return -1
    [n, m] = res;
    res = Check_Array(a1)
    if res == -1:
        print("MatrixArrayProduct, a1 error")
        return -1
    if res != m:
        print("MatrixArrayProduct, a1 error")
        return -1

    res = [0]*n

    for i in range(0,n):
        for j in range(0,m):
            res[i] += m1[i][j] * a1[j]

    return res

def dotProduct(a1, a2):
    "Restituisce il prodotto scalare di due vettori"
    n = Check_Array(a1)
    if n == -1:
        print("dotProduct, input error")
        return -1
    m = Check_Array(a2)
    if m != n:
        print("dotProduct, input error")
        return -1
    m = 0
    for i in range(0,n):
        m += a1[i] * a2[i]
        pass
    return m

def crossProduct(a1, a2):
    "Returns the cross product between two arrays in three dimensions (x, y, z)"
    n = Check_Array(a1)
    if n != 3:
        print("crossProdut, input error")
        return -1
    m = Check_Array(a2)
    if m != n:
        print("crossProdut, input error")
        return -1

    res = [0, 0, 0]
    res[0] = a1[1] * a2[2] - a1[2] * a2[1]
    res[1] = a1[2] * a2[0] - a1[0] * a2[2]
    res[2] = a1[0] * a2[1] - a1[1] * a2[0]

    return res

def SumOfSquares(input):
    "Returns the sum of squares of an array (List)"
    n = Check_Array(input)
    if n == -1:
        print("SumOfSquares, input error")
        return -1

    sum = 0
    for i in range(0,n):
        sum += input[i]**2

    return sum

def SumOfSquaresDifference(a1, a2):
    "Returns the sum of squares of the difference of two arrays (Lists)"
    n = Check_Array(a1)
    if n == -1:
        print("SumOfSquares, input error")
        return -1
    m = Check_Array(a2)
    if m != n:
        print("dotProduct, input error")
        return -1

    sum = 0
    for i in range(0,n):
        sum += (a1[i] - a2[i])**2

    return sum
