import numpy as np
from open3d import *
import copy
from random import randint
from random import random
from time import time
import math
from queue import Queue


def vecFromPoints(p1,p2):
    return np.array([p2[i]-p1[i] for i in range(len(p1))])


def rotation_matrix(axis, theta):
    axis = np.asarray(axis)
    axis = axis / math.sqrt(np.dot(axis, axis))
    a = math.cos(theta / 2.0)
    b, c, d = -axis * math.sin(theta / 2.0)
    aa, bb, cc, dd = a * a, b * b, c * c, d * d
    bc, ad, ac, ab, bd, cd = b * c, a * d, a * c, a * b, b * d, c * d
    return np.array([[aa + bb - cc - dd, 2 * (bc + ad), 2 * (bd - ac)],
                     [2 * (bc - ad), aa + cc - bb - dd, 2 * (cd + ab)],
                     [2 * (bd + ac), 2 * (cd - ab), aa + dd - bb - cc]])
    
    
    
def minus(v1,v2):
    return np.array([v1[i]-v2[i] for i in range(len(v1))])

def plus(v1,v2):
    return np.array([v1[i]+v2[i] for i in range(len(v1))])



def times(vect,t):
    return np.array([t*i for i in vect])

def hashlist(l):
    return str(l[0]) + "-" + str(l[1])




def make_base(triplet):
    x = triplet[0]
    y = triplet[1]
    z = triplet[2]

def intersectCandidates(base,toTest):
    exists = []
    noxists = []
    for i in range(len(toTest)):
        if toTest[i] in base:
            exists.append(i)
        else:
            noxists.append()

    return exists + noxists

def getCoordinates(base,point):
    return np.array([np.dot(b,point) for b in base])


class Surface:
    def __init__(self,mesh,triangle):
        self.mesh = mesh
        self.triangles = [triangle]
        self.frontier = self.mesh.triangleEdges[triangle][:]
        self.edges = self.mesh.triangleEdges[triangle][:]
        self.edgeTriangleCosts = {}
        self.edgeTriangles=[]
        self.computeEdgeTriangleCosts()
    
    def addTriangle(self,triangle):
        if triangle in self.triangles:
            #print("a")
            return False
        else:
            edges = self.mesh.triangleEdges[triangle]
            nbIn = 0
            toAppend = []
            toRemove = []
            for e in edges:
                if e in self.edges:
                    nbIn += 1
                    toRemove.append(e)
                else:
                    toAppend.append(e)
            #print(nbIn)
            if nbIn == 0:
                #print("b")
                return False
            elif nbIn == 3:
               # print("c")
                return False
            else:
                frontier = self.frontier[:]
                for e in toRemove:
                    frontier.remove(e)
                for e in toAppend:
                    frontier.append(e)
                #print(frontier)
                a = {}
                for e in frontier:
                    edge = mesh.edges[e][0]
                    try:
                        a[edge[0]] += 1
                    except KeyError:
                        a[edge[0]] = 1
                    if len(edge) > 1:
                        try:
                            a[edge[1]] += 1
                        except KeyError:
                            a[edge[1]] = 1
                #print(a)
                for k in a:
                    if a[k] > 2:
                        #print(a[k])
                        return False
                        
                self.frontier = frontier
                self.triangles.append(triangle)
                self.edges = self.edges + toAppend
                self.computeEdgeTriangleCosts()
                return True
            
    def addFinalTriangle(self,triangle):
        edges = self.mesh.triangleEdges[triangle]
        toAppend = []
        toRemove = []
        for e in edges:
            if e in self.edges:
                toRemove.append(e)
            else:
                toAppend.append(e)        
        frontier = self.frontier[:]
        for e in toRemove:
            frontier.remove(e)
        for e in toAppend:
            frontier.append(e)
        self.frontier = frontier
        self.triangles.append(triangle)
        self.edges = self.edges + toAppend
        self.computeEdgeTriangleCosts()
        return True
            
    def computeEdgeTriangleCosts(self):
        self.edgeTriangles = []
        for e in self.frontier:
           
            triangles = self.mesh.edges[e][0]
            if len(triangles) > 1:
                cost = abs(self.mesh.triangleAngle(triangles[0],triangles[1]))
                self.edgeTriangleCosts[triangles[0]] = cost
                self.edgeTriangleCosts[triangles[1]] = cost
                if triangles[0] in self.triangles:
                    self.edgeTriangles.append(triangles[1])
                else:
                    self.edgeTriangles.append(triangles[0])
    
    def sortTriangles(self):
        
        def sort_key(t):
            return self.edgeTriangleCosts[t]
        
        self.edgeTriangles.sort(key=sort_key)

class TriangularMesh:
    def __init__(self,mesh):
        self.mesh = copy.deepcopy(mesh)
        self.vertices = np.asarray(mesh.vertices)[:]
        self.triangles = np.asarray(mesh.triangles)[:]
        self.edges = []
        self.triangleEdges = [[] for t in self.triangles]
        self.verticeEdges = [[] for v in self.vertices]
        self.computeEdges()
        
 
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
            if k % (p//(n*100)) == 0:
                print(str(k) + " / " + str(p//n))
                print(time()-t1)
                t1=time()
        print("Final steps...")
        while self.finalSurfaceStep():
            k += 1
            if k % (p//(n*100)) == 0:
                print(str(k) + " / " + str(p//n))
                print(time()-t1)
                t1=time()

        
    

    def printColors(self):
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

        draw_geometries(P)
        print(np.asarray(P[1].triangles))
        return Q
    
mesh = TriangularMesh(read_triangle_mesh("Open3D/examples/TestData/bun_zipper_res4.ply"))



class Parameterization:
    def __init__(self,mesh,surface):
        self.mesh = mesh
        self.isIn = [False for t in self.mesh.triangles]
        for t in self.mesh.surfaces[surface].triangles:
            self.isIn[t] = True
        self.bases = [[] for t in self.mesh.triangles]
        self.pred = [-1 for t in self.mesh.triangles]
        self.toProcess = Queue()
        self.seed = self.mesh.surfaces[surface].triangles[0]
        
        
    def voisins(self,triangle):
        edges = self.mesh.triangleEdges[triangle]
        toAppend = []
        for e in edges:
            toAppend = toAppend + self.mesh.edges[e][0]
        for t in toAppend:
            if self.pred[t] == -1 and self.isIn[t]:
                self.pred[t] = triangle
                self.toProcess.put(t)
        return True
        

    
    def getInitialBase(self):
        points = self.mesh.triangles[self.seed]
        x = vecFromPoints(self.mesh.vertices[points[0]],self.mesh.vertices[points[1]])
        y = vecFromPoints(self.mesh.vertices[points[0]],self.mesh.vertices[points[2]])
        z = np.cross(x,y)
        
        self.bases[seed] = np.array(x,y,z)
        
        
    def getBaseFromPrevious(self,current,previous):
        previousBase = self.bases[previous]
        pNormal = previousBase[2]
        
        points = self.mesh.triangles[current]
        previouspoints = self.mesh.triangles[previous]
        
        points = intersectCandidates(previouspoints,points)
        
        previouspoints = intersectCandidates(points,previouspoints)
        
        previouspoints[0] = points[0]
        previouspoints[1] = points[1]
        
        x = vecFromPoints(self.mesh.vertices[points[0]],self.mesh.vertices[points[1]])
        y1 = vecFromPoints(self.mesh.vertices[points[0]],self.mesh.vertices[previouspoints[2]])
        y2 = vecFromPoints(self.mesh.vertices[points[0]],self.mesh.vertices[points[2]])
        
    
        
        
        
        
        
        x = vecFromPoints(self.mesh.vertices[points[0]],self.mesh.vertices[points[1]])
        y = vecFromPoints(self.mesh.vertices[points[0]],self.mesh.vertices[points[2]])
        z = np.cross(x,y)
        

    def getBases(self):
        self.pred[self.seed] = self.seed
        self.getInitialBase()
        self.voisins(seed)
        
    
        
        
        while self.toProcess.empty == False:
            triangle = self.toProcess.get()
            
            self.bases[triangle] = self.getBaseFromPrevious(triangle,self.pred[triangle])
            
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
 
