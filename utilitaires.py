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

    
def triangleWeights(points):
    u1 = points[0]
    u2 = points[1]
    u3 = points[2]
    
    A = 0
    
    A += u1[0]*u2[1] - u2[0]*u1[1]
    A += u2[0]*u3[1] - u3[0]*u2[1]
    A += u3[0]*u1[1] - u1[0]*u3[1]
    
    W1 = np.complex(u3[0] - u2[0],u3[1]-u2[1])
    W2 = np.complex(u1[0] - u3[0],u1[1]-u3[1])
    W3 = np.complex(u2[0] - u1[0],u2[1]-u1[1])
    
    return [W1,W2,W3,abs(A)]


    
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
    
    
    z = times(z,1/np.linalg.norm(z))
    
    x = times(x,1/np.linalg.norm(x))


    y = minus( y, times(x,np.dot(y,x)  )      )
    
    y = times(y,1/np.linalg.norm(y))
    
    y = np.cross(z,x)
    
    #print(z)
    #print(x,y,z)
    return copy.deepcopy(np.array([x,y,z]))

def intersectCandidates(base,toTest):
    exists = []
    noxists = []
    #print(base)
    for i in range(len(toTest)):
        if toTest[i] in base:
            exists.append(i)
        else:
            noxists.append(i)

    return exists + noxists

def getCoordinates(base,point):
    return np.array([np.dot(b,point) for b in base])