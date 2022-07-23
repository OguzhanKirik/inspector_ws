import open3d as o3d
import numpy as np

def ReadPointCloudInputTextFile( filePath):
        '''
        Point cloud: X as first row, Y as second row, Z as matrix.
        Returns a standard point cloud matrix ([[x1, y1, z1], [x2, y2, z2], ...]
        '''

        file = open(filePath,"r")
        line = file.readline()
        XX = []
        YY = []
        ZZ = []
        all = []
        id = 0
        while line != "":
            divisi = line.split("\t")
            # if XX == []:
            #     Profile0 = [float(ele) for ele in divisi]
            #     numX = len(Profile0)* xScale
            #     XX = list(np.arange(-numX / 2, numX / 2, xScale))
            all.append([float(ele) for ele in divisi])
            # pass

            line = file.readline()
            id = id + 1
        XX = all[-1]
        ZZ = all[0:-1]
        YY= list(np.zeros(len(ZZ)))
        file.close()
        #File loading of "old point cloud" *****************************************************************************

        return [XX, YY, ZZ]

def ReadRobotCoordinateInputFile(filePath : str):
        '''
        Returns an array containig:
        Num - progressive number of acquired coordinate/profilometer trigger
        Coord - [X, Y, Z] coordinate of robot tool
        Quat - [q0, qx, qy, qz] coordinate of robot tool
        '''
        #File loading of robot coordinate ******************************************************************************
        file = open(filePath,"r")
        line = file.readline() #reads the header

        Num = []
        Coord = []
        Quat = []

        line = file.readline()
        id = 0
        while line != "":
            divisi = line.split("\t")
            Num.append(int(divisi[0]))
            rX = (float(divisi[1]))
            rY = (float(divisi[2]))
            rZ = (float(divisi[3]))
            rqw = (float(divisi[4]))
            rqx = (float(divisi[5]))
            rqy = (float(divisi[6]))
            rqz = (float(divisi[7]))
            Coord.append([rX, rY, rZ])
            Quat.append([rqw, rqx, rqy, rqz])
            line = file.readline()
            id = id + 1
        file.close
        #File loading of robot coordinate ******************************************************************************

        return [Num, Coord, Quat]
