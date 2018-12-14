import numpy as np
from open3d import *
import copy
from random import randint
from random import random
from time import time
import math
from queue import Queue

from utilitaires import *
from Surface import *
from TriangularMesh import *

class Parameterization:
    def __init__(self,mesh,surface):
        self.mesh = mesh
        
        self.mesh.surfaces[surface].triangles = self.mesh.surfaces[surface].triangles[:]
        self.isIn = [False for t in self.mesh.triangles]
        for t in self.mesh.surfaces[surface].triangles:
            self.isIn[t] = True
        self.bases = [[] for t in self.mesh.triangles]
        self.pred = [-1 for t in self.mesh.triangles]
        self.toProcess = Queue()
        self.seed = self.mesh.surfaces[surface].triangles[0]
        self.pointsInnerCoordinates=[[] for i in self.mesh.vertices]
        self.triangles = self.mesh.surfaces[surface].triangles[:]
        self.surface = surface
        self.getVertices()
        self.weights = [[] for t in self.mesh.triangles]
        
        
        
    def getVertices(self):
        s = set()
        for t in self.triangles:
            s.update(self.mesh.triangles[t])
                
        self.vertices = list(s)
        
        
        
        
        q = set()
        for t in self.mesh.surfaces[self.surface].frontier:
            q.update(self.mesh.edges[t][1])
            
        edgeVertices = list(q)
        
        a = edgeVertices[0]
        b = edgeVertices[-1]
    
        
        self.vertices.remove(a)
        self.vertices.append(a)
        self.vertices.remove(b)
        
        self.vertices.append(b)
        
    def voisins(self,triangle):
        edges = self.mesh.triangleEdges[triangle]
        toAppend = []
        for e in edges:
            toAppend = toAppend + self.mesh.edges[e][0]
        #print(toAppend)
        for t in toAppend:
            if self.pred[t] == -1 and self.isIn[t]:
                #print(self.toProcess.qsize())
                self.pred[t] = triangle
                self.toProcess.put(t)
        return True
        

    
    def getInitialBase(self):
        points = self.mesh.triangles[self.seed]
        x = vecFromPoints(self.mesh.vertices[points[0]],self.mesh.vertices[points[1]])
        y = vecFromPoints(self.mesh.vertices[points[0]],self.mesh.vertices[points[2]])
        z = np.cross(x,y)
        
        self.bases[self.seed] = make_base([x,y,z])
        
        
    def getBaseFromPrevious(self,current,previous):
        previousBase = self.bases[previous][:]
        
        pointsindex = self.mesh.triangles[current][:]
        previouspointsindex = self.mesh.triangles[previous][:]
        
        points = [self.mesh.triangles[current][k] for k in intersectCandidates(previouspointsindex,pointsindex)]
        
        previouspoints = [self.mesh.triangles[previous][k] for k in intersectCandidates(pointsindex,previouspointsindex)]
        
        
        #print(previouspoints[0],previouspoints[1],points[0],points[1])
        
        if previouspoints[0] != points[0] and previouspoints[1] != points[1]:
            print(previouspoints[0],previouspoints[1],points[0],points[1])
        previouspoints[0] = points[0]
        previouspoints[1] = points[1]
        
        
        x = vecFromPoints(self.mesh.vertices[points[0]],self.mesh.vertices[points[1]])
        y1 = vecFromPoints(self.mesh.vertices[points[0]],self.mesh.vertices[previouspoints[2]])
        y2 = vecFromPoints(self.mesh.vertices[points[0]],self.mesh.vertices[points[2]])
        
    
        tempz = np.cross(y1,x)

        if np.dot(tempz,previousBase[2]) > 0:
            z = np.cross(y2,x)
        else:
            z = np.cross(x,y2)
        
        #print(points)
        return make_base([x[:],y2[:],z[:]])
        

    def getBases(self):
        self.pred[self.seed] = self.seed
        self.getInitialBase()
        self.voisins(self.seed)
        
    
        
        
        while self.toProcess.empty() == False:
            triangle = self.toProcess.get()
            #print(self.toProcess.qsize())
            self.bases[triangle] = self.getBaseFromPrevious(triangle,self.pred[triangle])
            self.voisins(triangle)
            
        
#        
#        for i in self.triangles:
#            self.pointsInnerCoordinates[self.mesh.triangles[i][0]] = getCoordinates(self.bases[i],self.mesh.vertices[self.mesh.triangles[i][0]])
#            self.pointsInnerCoordinates[self.mesh.triangles[i][1]] = getCoordinates(self.bases[i],self.mesh.vertices[self.mesh.triangles[i][1]])
#            self.pointsInnerCoordinates[self.mesh.triangles[i][2]] = getCoordinates(self.bases[i],self.mesh.vertices[self.mesh.triangles[i][2]])
#        
#        
#        
            
    def getM(self):
        
        
        for t in self.triangles:
            self.weights[t] = triangleWeights(   [  getCoordinates(self.bases[t],self.mesh.vertices[self.mesh.triangles[t][0]]) ,
                                                getCoordinates(self.bases[t],self.mesh.vertices[self.mesh.triangles[t][1]]) ,
                                                getCoordinates(self.bases[t],self.mesh.vertices[self.mesh.triangles[t][2]]) ] )
        
        #print(self.weights)
        
        n = len(self.vertices)
        p = len(self.triangles)
        
        self.M = np.array([[np.complex(0,0) for j in range(p)] for j in range(n)])
        
        for j in range(p):
            for i in range(n):
                
                t = self.triangles[j]
                v = self.vertices[i]
                
                if v in self.mesh.triangles[t]:
                    k = list(self.mesh.triangles[t]).index(v)
                    #print(1/np.sqrt(self.weights[t][3]) * self.weights[t][k])
                    #print(self.weights[t][3],self.weights[t][k])
                    #print(np.sqrt(self.weights[t][3]))
                    self.M[i][j] = 1/np.sqrt(self.weights[t][3]) * self.weights[t][k]
                    #print(self.M[i][j])
                    
        self.M = self.M.transpose()          
    def getAandb(self,p,u):
        n = len(self.vertices)
        
        m = len(self.triangles)
        self.up = u
        
        self.A = np.array([[0.0 for j in range(2*(n-p))] for j in range(2*m)])
        
        for i in range(m):
            for j in range(n-p):
                self.A[i][j] = self.M[i][j].real
                self.A[i][n-p+j] = -self.M[i][j].imag
                self.A[m+i][j] = self.M[i][j].imag
                self.A[m+i][n-p+j] = self.M[i][j].real
                
                
                
        bmat = np.array([[0.0 for j in range(2*p)] for j in range(2*m)])
        for i in range(m):
            for j in range(p):
                bmat[i][j] = -self.M[i][n-j-1].real
                bmat[i][p+j] = self.M[i][n-j-1].imag
                bmat[m+i][j] = -self.M[i][n-j-1].imag
                bmat[m+i][p+j] = -self.M[i][n-j-1].real
                
        Up = np.array([u[i].real for i in range(p)] + [u[i].imag for i in range(p)])
        
                
        self.b = np.dot(bmat,Up)
        
        
    def getFinalU(self):
        print(self.b)
        temp = list(np.linalg.lstsq(self.A,self.b))[0]
        print(np.linalg.norm(np.dot(self.A,temp) - self.b))
        self.U = []
        
        for i in range(len(temp)//2):
            self.U.append(np.complex(temp[i],temp[len(temp)//2 + i]))
                
        self.U = self.U + self.up
        

        
        
    def printSurface(self):
        P = copy.deepcopy(self.mesh.mesh)

        P.triangles = Vector3iVector(np.array([P.triangles[k] for k in self.triangles]))
        P.triangle_normals = Vector3dVector(np.asarray([P.triangle_normals[k] for k in self.triangles]))
        
        for i in range(len(self.vertices)):
            P.vertices[self.vertices[i]] = np.array([self.U[i].real,self.U[i].imag,0])

        P.compute_triangle_normals()
        P.compute_vertex_normals()
        draw_geometries([P])
        return True