import numpy as np
from open3d import *
import copy
from random import randint
from random import random
from time import time
import math
from queue import Queue

from utilitaires import *

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
                    edge = self.mesh.edges[e][0]
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