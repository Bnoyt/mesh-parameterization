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
from Parameterization import *

                
                
                
u = [0j, (500+500j)]   
mesh = TriangularMesh(read_triangle_mesh("bun_zipper_res4.ply"))
draw_geometries([mesh.mesh])
mesh.stepUntilDeath(10)
#mesh.surfaces[3].triangles = mesh.surfaces[3].triangles[:5]
mesh.printColors(surface=3)

p = Parameterization(mesh,3)      
p.getBases()
p.getM()
p.getAandb(2,u)
p.getFinalU()
p.printSurface()
for t in p.triangles:
    #print( p.bases[t][2])
    #print(np.dot(p.mesh.mesh.triangle_normals[t],p.bases[t][2]))
    p.mesh.mesh.triangle_normals[t] = p.bases[t][2]

#draw_geometries([p.mesh.mesh])   
 
