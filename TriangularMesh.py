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

class TriangularMesh:
    def __init__(self,mesh):
        self.mesh = copy.deepcopy(mesh)
        self.vertices = np.asarray(mesh.vertices)[:]
        self.triangles = np.asarray(mesh.triangles)[:]
        self.edges = []
        self.triangleEdges = [[] for t in self.triangles]
        self.verticeEdges = [[] for v in self.vertices]
        self.computeEdges()
        self.mesh.compute_triangle_normals()
        self.mesh.compute_vertex_normals()
        
 
    def computeEdges(self):
        a = {}
        for i in range(len(self.triangles)):
            t = self.triangles[i]
            t.sort()
            try:
                a[hashlist([t[0],t[1]])].append(i)
            except KeyError:
                a[hashlist([t[0],t[1]])] = [i]
            try:
                a[hashlist([t[1],t[2]])].append(i)
            except KeyError:
                a[hashlist([t[1],t[2]])] = [i]
            try:
                a[hashlist([t[0],t[2]])].append(i)
            except KeyError:
                a[hashlist([t[0],t[2]])] = [i]
           
            
        for k in a:
            p = k.split("-")
            p[0] = int(p[0])
            p[1] = int(p[1])
            self.edges.append([a[k],p])
            self.triangleEdges[a[k][0]].append(len(self.edges)-1)
            if len(a[k])>1:
                self.triangleEdges[a[k][1]].append(len(self.edges)-1)
        
        for i in range(len(self.edges)):
            v = self.edges[i][1]
            self.verticeEdges[v[0]].append(i)
            self.verticeEdges[v[1]].append(i)
            
    def triangleAngle(self,t1,t2):
        n1 = self.triangleNormals[t1]
        n2 = self.triangleNormals[t2]
        return np.arccos(np.dot(n1,n2))
    
    
            
    def generateSurfaces(self,n):
        self.mesh.compute_triangle_normals()
        self.triangleNormals = np.asarray(self.mesh.triangle_normals)
        
        self.isTriangleFree = [True for i in self.triangles]
        
        p = len(self.triangles)
        randomSeeds = [randint(0,p-1)]
        for i in range(n-1):
            q = randint(0,p-1)
            while q in randomSeeds:
                q = randint(0,p-1)
            randomSeeds.append(q)
            
        for t in randomSeeds:
            self.isTriangleFree[t] = False
        
        self.surfaces = [Surface(self,q) for q in randomSeeds]

                    
    def surfaceStep(self):
        newTriangleFound = False
        for s in self.surfaces:
            s.sortTriangles()
            
            for t in s.edgeTriangles:
                if self.isTriangleFree[t]:
                    if s.addTriangle(t):
                        self.isTriangleFree[t] = False
                        newTriangleFound = True
                        break
                
        return newTriangleFound

    def finalSurfaceStep(self):
        newTriangleFound = False
        for s in self.surfaces:
            s.sortTriangles()
            for t in s.edgeTriangles:
                if self.isTriangleFree[t]:
                    if s.addFinalTriangle(t):
                        self.isTriangleFree[t] = False
                        newTriangleFound = True
                        break
                
        return newTriangleFound
                    
        
        
        
    def stepUntilDeath(self,n):
        self.generateSurfaces(n)
        k = 0
        p = len(self.triangles)
        t1 = time()
        print("Normal steps...")
        while self.surfaceStep():
            k += 1
            if (k*n) % (p//100) == 0:
                print(str(k) + " / " + str(p//n))
                print(time()-t1)
                t1=time()
        print("Final steps...")
        while self.finalSurfaceStep():
            k += 1
            if (k*n) % (p//100) == 0:
                print(str(k) + " / " + str(p//n))
                print(time()-t1)
                t1=time()

        
    

    def printColors(self,surface = -1):
        Q = 0
        self.mesh.compute_vertex_normals()
        self.mesh.compute_triangle_normals()
        P = [copy.deepcopy(self.mesh) for s in self.surfaces]

        for i in range(len(self.surfaces)):
            P[i].triangles = Vector3iVector(np.array([P[i].triangles[k] for k in self.surfaces[i].triangles]))
            P[i].triangle_normals = Vector3dVector(np.asarray([P[i].triangle_normals[k] for k in self.surfaces[i].triangles]))
        for k in P:
            k.paint_uniform_color([random(), random(), random()])
            Q += len(np.asarray(k.triangles))

        if surface==-1:
            
            draw_geometries(P)
        else:
            draw_geometries([P[surface]])
        #print(np.asarray(P[1].triangles))
        return Q
    

