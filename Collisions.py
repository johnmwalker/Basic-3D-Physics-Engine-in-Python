# -*- coding: utf-8 -*-
"""
Created on Wed Dec  6 20:45:16 2017

@author: John
"""
import numpy as np
import Vector
import Search
import Error
import copy as cp
import matplotlib
from sympy import Plane, Point3D, N
import pybullet as pyb
import euclid3

COLTOL = 10**-3
#COLTOL = 1
LINDRAGCOEF = 0.25
COEFRESTITUTION = 0.8
COEFFRICTION = 0.1

PENETRATING = -1
COLLISION = 0
NOCOLLISION = 1

class CollisionHandler(object):
    """ This class handles all collisions. Initialize and then call checkCollisions()
    every frame.
    """
    def __init__(self, listOfEntities, search, l,r,t,b,f,a,g):
        
        self.listOfEntities = listOfEntities
        self.l = l # left wall position
        self.r = r # right wall position
        self.t = t # top wall position
        self.b = b # bottom wall position
        self.f = f # fore wall position
        self.a = a # aft wall position
        
        self.g = g # gravity boolean
        
        if search == 'newton':
            print("Search is Newton")
            self.search = Search.Newton.find
            
        self.physicsClient = pyb.connect(pyb.DIRECT)
        self.listOfCents = []
        for ent in self.listOfEntities:
            if ent.entityType is "box":
                cid = pyb.createCollisionShape(pyb.GEOM_BOX,
                    halfExtents=ent.size)
                cent = pyb.createMultiBody(baseCollisionShapeIndex = cid)
                pyb.resetBasePositionAndOrientation(cent, ent.body.pos.v, ent.body.q.asArrayForPyBullet())
                self.listOfCents.append(cent)
                
            if ent.entityType is "ball":
                cid = pyb.createCollisionShape(pyb.GEOM_SPHERE,
                                                     radius = ent.boundRadius)
                cent = pyb.createMultiBody(baseCollisionShapeIndex = cid)
                pyb.resetBasePositionAndOrientation(cent, ent.body.pos.v, ent.body.q.asArrayForPyBullet())
                self.listOfCents.append(cent)
                
        if self.l is not None:
            xSize = (self.r - self.l)/2
            ySize = (self.t - self.b)/2
            zSize = (self.f - self.a)/2
            cid = pyb.createCollisionShape(pyb.GEOM_BOX, halfExtents=[xSize,ySize,zSize])
            
            linkPos = [[xSize*4,0,0], [xSize*2,ySize*2,0], [xSize*2,-ySize*2,0],
                       [xSize*2,0,zSize*2], [xSize*2,0,-zSize*2]]

            self.wallBody = pyb.createMultiBody(1,cid,
                    linkMasses                    = [1.0]*5,
                    linkCollisionShapeIndices     = [cid]*5,
                    linkVisualShapeIndices        = [-1]*5,
                    linkPositions                 = linkPos, 
                    linkOrientations              = [[0,0,0,1]]*5,
                    linkInertialFramePositions    = [[0,0,0]]*5,
                    linkInertialFrameOrientations = [[0,0,0,1]]*5,
                    linkParentIndices             = [0]*5,
                    linkJointTypes                = [pyb.JOINT_FIXED]*5,
                    linkJointAxis                 = [[1,0,0]]*5
                    )
            pyb.resetBasePositionAndOrientation(self.wallBody, [-xSize*2,0,0], [0,0,0,1])
            print('wallBody is ' + str(self.wallBody))
            
                
            
                
    def checkColPybNoPenetration(self, App, backupEntities):
        """
        """
        self.listOfCollisions = []
        tryAgain = False
        statusList = [NOCOLLISION]
        cents = self.listOfCents
        
        for i, ent in enumerate(self.listOfEntities):
            pyb.resetBasePositionAndOrientation(cents[i], ent.body.x.v, ent.body.q.asArrayForPyBullet())

        for i, ent1 in enumerate(self.listOfEntities):
            
            # Check for collision between bodies
            for j, ent2 in enumerate(self.listOfEntities[i+1:]):
                k = i+j+1

                colPoints = pyb.getClosestPoints(cents[i],cents[k],COLTOL)
                if colPoints is not ():
                    for col in colPoints:
                        statusList.append(self.isCollidingPybullet(i,k,col))                    

            # Check for collision with walls
#            statusList.append(self.checkWallCollisions(i))
                
        if min(statusList) is COLLISION:
            self.resolveCollisions()
            tryAgain = False
        elif min(statusList) is PENETRATING:

#            self.resolveCollisions()
            pError = Error.penetrationErrorSetup(App, backupEntities, self.closestDistance)
            self.search(pError, App.currentTime - App.stepsize/2, COLTOL, 1)
            tryAgain = True
            
        return tryAgain
                
    def checkCollisionsPybullet(self):
        """
        """
        self.listOfCollisions = []
        tryAgain = False
        statusList = [NOCOLLISION]
        cents = self.listOfCents
        self.collisionPairs = []
        
        for i, ent in enumerate(self.listOfEntities):
            pyb.resetBasePositionAndOrientation(cents[i], ent.body.x.v, ent.body.q.asArrayForPyBullet())

        for i, ent1 in enumerate(self.listOfEntities):
            
            # Check for collision between bodies
            for j, ent2 in enumerate(self.listOfEntities[i+1:]):
                k = i+j+1

                colPoints = pyb.getClosestPoints(cents[i],cents[k],COLTOL)
                if colPoints is not ():
                    for col in colPoints:
                        statusList.append(self.isCollidingPybullet(i,k,col))  

            # Check for collision with walls
            if self.l is not None:
                colPointsWall = pyb.getClosestPoints(cents[i],self.wallBody,COLTOL)
                if colPointsWall is not ():
                    for col in colPointsWall:
                        statusList.append(self.isCollidingPybullet(i,-1,col))
                
        if min(statusList) is COLLISION:
            self.resolveCollisions()
            tryAgain = False
        elif min(statusList) is PENETRATING:
            tryAgain = True
            self.resolveCollisions()
            
        return tryAgain
    
    def checkCollisions(self):
        """
        """
        self.listOfCollisions = []
        tryAgain = False
        statusList = [NOCOLLISION]
        
        for i, ent1 in enumerate(self.listOfEntities):
            
            # Check for collision between bodies
            for j, ent2 in enumerate(self.listOfEntities[i+1:]):
                k = i+j+1
                # First, do a bounding sphere check
                d = ent1.body.x.v - ent2.body.x.v
                dMag = np.linalg.norm(d)
                if dMag < ent1.boundRadius + ent2.boundRadius + COLTOL:
                    
                    if ent1.entityType is 'box' and ent2.entityType is 'box':
                        
#                        print('box-box collision')

                        # Check each vertex
                        statusList.append(self.checkBoxCollisions(i, k))
                    elif ent1.entityType is 'ball' and ent2.entityType is 'ball':
                        
                        # Just check for penetration vs collision
                        statusList.append(self.checkSphereCollisions(i,k))
#                        print('sphere-sphere collision')
                    
                    else:
#                        print('sphere-box collision, unhandled')
                        # Check each vertex against the sphere
#                        statusList.append(self.checkBoxSphereCollision(i, k))
                        asdf = 0

            # Check for collision with walls
#            statusList.append(self.checkWallCollisions(i))
                
        if min(statusList) is COLLISION:
            self.resolveCollisions()
            tryAgain = False
        elif min(statusList) is PENETRATING:
#            tryAgain = True
            self.resolveCollisions()
            
        return tryAgain
            
    def isColliding(self, vert, ent1Index, ent2Index, norm): #u, v
        """Check if two bodies are colliding with each other.
        """
        
        ent1 = self.listOfEntities[ent1Index]
        ent2 = self.listOfEntities[ent2Index]
        status = NOCOLLISION
        
        pt1 = vert - ent1.body.x.v
        pt2 = vert - ent2.body.x.v
        
        vel1 = ent1.body.velocity.v + np.cross(ent1.body.omega.v, pt1)
        vel2 = ent2.body.velocity.v + np.cross(ent2.body.omega.v, pt2)
        
#        norm = np.cross(u,v)
#        norm = -norm/np.linalg.norm(norm)
        norm = norm/np.linalg.norm(norm)
        
        Vr = vel1 - vel2
        Vrn = np.dot(Vr, norm)
        
        if Vrn < 0:
            self.listOfCollisions.append(Collision(ent1Index,ent2Index,norm,vert,Vr,-(Vr - (np.dot(np.dot(Vr,norm),norm)))))
            status = COLLISION

        return status
    
    def isCollidingPybullet(self, ent1Index, ent2Index, colData):
        """
        """
#        if colData[8]<-COLTOL:
#            print(colData[8])
#            return PENETRATING
        
#        colData = colData[0]
#        contactFlag = colData[0]
#        idA = colData[1]
#        idB = colData[2]
#        linkIndex1 = colData[3]
#        linkIndex2 = colData[4]
        pt1 = np.array(colData[5])
        pt2 = np.array(colData[6])
        dist = colData[8]
        normalForce = colData[9]
        
        status = NOCOLLISION
        ent1 = self.listOfEntities[ent1Index]
        
        pt1FromBody = pt1-ent1.body.x.v
        pt1 = ent1.body.x.v + pt1FromBody
        
        if ent2Index is not -1:
            ent2 = self.listOfEntities[ent2Index]
        
            if ent1.entityType is 'box' and ent2.entityType is 'box':
                n = pt1-pt2
                if dist < 0:
                    n *= -1
                
            else: 
                n = colData[7] # Check whether this is actually what I want
                
            pt2FromBody = pt2-ent2.body.x.v
            relVel = ent1.body.v.v - ent2.body.v.v
            tangent = -np.cross(ent1.body.omega.v, pt1FromBody) + np.cross(ent2.body.omega.v, pt2FromBody)

        else:
            if ent1.entityType is 'box':
                n = pt1-pt2
                if dist < 0:
                    n *= -1
                
            else: 
                n = colData[7] # Check whether this is actually what I want
                
            tangent = -np.cross(ent1.body.omega.v, pt1FromBody)
            relVel = ent1.body.v.v

        nNorm = n/np.linalg.norm(n)
#        print(nNorm)

#        pt2 = ent2.body.x.v + pt2FromBody
        
        Vrn = np.dot(relVel, nNorm)
        if Vrn < 0:
            
            status = COLLISION
            self.listOfCollisions.append(Collision(ent1Index, ent2Index, nNorm, pt1, relVel, tangent, dist))
            
#            if dist<-COLTOL:
#                status = PENETRATING
            
#            if dist < 0:
#                repulsiveForce  = Vector.Quaternion(0,nNorm*dist)*1000
#                ent1.body.force = repulsiveForce
#                ent2.body.force = -1*repulsiveForce
                
        return status
    
    def closestDistance(self):
        """
        """
        listOfDistances = []
        
        for collision in self.listOfCollisions:
            
            listOfDistances.append(collision.dist)
        
        return max(listOfDistances)
    
    def isCollidingWithWall(self, vert, ent1Index, ent2, u, v):
        """Check if a body is colliding with a wall.
        """
        status = NOCOLLISION
        ent1 = self.listOfEntities[ent1Index]
        
        pt = vert - ent1.body.x.v
        
        vel = ent1.body.velocity.v + np.cross(ent1.body.omega.v, pt)
        
#        vel = QVRotation(ent.body.q,vel)
        
        n = np.cross(u,v)
        n = n/np.linalg.norm(n)
        
        Vr = vel
        Vrn = np.dot(Vr, n)
        
        if Vrn < 0:
            self.listOfCollisions.append(Collision(ent1Index,ent2,n,vert,Vr,-(Vr - (np.dot(np.dot(Vr,n),n)))))
            status = COLLISION
            
        return status
    
    def getClosestThreePoints(self, ent1, ent2, pt):
        """
        """
        v1 = ent1.vertices
        v2 = ent2.vertices
        pt = np.array(pt)
        
        pointOnEnt1 = np.where(np.linalg.norm(pt-v1)<0.1)
        pointOnEnt2 = np.where(np.linalg.norm(pt-v2)<0.1)
        
        if pointOnEnt1:
            relv = v1-pt
            relvmag = np.array([np.linalg.norm(v) for v in relv])
            relvsorted = np.sort(relvmag)
            threeClosest = relvsorted[0:4]
            
        elif pointOnEnt2:
            relv = v2-pt
            relvmag = np.array([np.linalg.norm(v) for v in relv])
            relvsorted = np.sort(relvmag)
            threeClosest = relvsorted[0:4]
            
        else:
            print("Collision point not found in either collision object.")
            
        return threeClosest
    
    def checkBoxCollisions(self, ent1Index, ent2Index):
        """
        """
        statusList = [NOCOLLISION]
        ent1 = self.listOfEntities[ent1Index]
        ent2 = self.listOfEntities[ent2Index]

        #Verticies in world frame
        v1 = ent1.vertices
        v2 = ent2.vertices
                
        for i in range(8):
            
#            inside = True; #Not sure what this is, maybe a diagnostic thing?
            
            #Front face of body 2:
            u = v2[1]-v2[0]
            v = v2[3]-v2[0]
            d = self.calcDistanceFromPointToPlane(v1[i], u, v, v2[0])
            
            if np.linalg.norm(d) < COLTOL:
                
                f = v2[0:4]
                if self.isPointOnFace(v1[i], f):
                    norm = np.cross(v,u)
                    statusList.append(self.isColliding(v1[i], ent1Index, ent2Index, norm))
                    
            # Aft face of body 2:
            u = v2[6]-v2[7]
            v = v2[4]-v2[7]
            d = self.calcDistanceFromPointToPlane(v1[i], u, v, v2[7])

            if np.linalg.norm(d) < COLTOL:
                
                f = v2[4:]
                if self.isPointOnFace(v1[i], f):
                    norm = np.cross(v,u)
                    statusList.append(self.isColliding(v1[i], ent1Index, ent2Index, norm))
    
            #Top face of body 2:
            u = v2[2]-v2[6]
            v = v2[5]-v2[6]
            d = self.calcDistanceFromPointToPlane(v1[i], u, v, v2[6])

            if np.linalg.norm(d) < COLTOL:
                
                f = [v2[k] for k in [6,2,1,5]]
                if self.isPointOnFace(v1[i], f):
                    norm = np.cross(v,u)
                    statusList.append(self.isColliding(v1[i], ent1Index, ent2Index, norm))
                    
            #Bottom face of body 2:
            u = v2[0]-v2[4]
            v = v2[7]-v2[4]
            d = self.calcDistanceFromPointToPlane(v1[i], u, v, v2[4])

            if np.linalg.norm(d) < COLTOL:

                f = [v2[k] for k in [4,0,3,7]]
                if self.isPointOnFace(v1[i], f):
                    norm = np.cross(v,u)
                    statusList.append(self.isColliding(v1[i], ent1Index, ent2Index, norm))
    
            #Left face of body 2:
            u = v2[5]-v2[4]
            v = v2[0]-v2[4]
            d = self.calcDistanceFromPointToPlane(v1[i], u, v, v2[4])

            if np.linalg.norm(d) < COLTOL:
                f = [v2[k] for k in [4,5,1,0]]

                if self.isPointOnFace(v1[i], f):
                    norm = np.cross(v,u)
                    statusList.append(self.isColliding(v1[i], ent1Index, ent2Index, norm))  
                    
            #Right face of body 2:
            u = v2[6]-v2[2]
            v = v2[3]-v2[2]
            d = self.calcDistanceFromPointToPlane(v1[i], u, v, v2[2])

            if np.linalg.norm(d) < COLTOL:
                
                f = [v2[k] for k in [2,6,7,3]]
                if self.isPointOnFace(v1[i], f):
                    norm = np.cross(v,u)
                    statusList.append(self.isColliding(v1[i], ent1Index, ent2Index, norm))    
                    
        # Now do edge-edge collisions
        uList = []
        vList = []
        neighbors = [1,1,4]
        start = [0,4,0]
        for n, s in zip(neighbors,start): # First do all lines bordering front face, then back face, then lines in between
            for i in range(4):
                a1 = euclid3.Point3(v1[i+s][0],v1[i+s][1],v1[i+s][2])
                b1 = euclid3.Point3(v1[i+n][0],v1[i+n][1],v1[i+n][2])
                uList.append(euclid3.LineSegment3(a1, b1))
                a2 = euclid3.Point3(v2[i+s][0],v2[i+s][1],v2[i+s][2])
                b2 = euclid3.Point3(v2[i+n][0],v2[i+n][1],v2[i+n][2])
                vList.append(euclid3.LineSegment3(a2, b2))
                
        for u in uList:
            for v in vList:
                c = u.connect(v)
                if c.length<COLTOL:
                    n = c.v
                    norm = np.array([n.x,n.y,n.z])
                    print("edges collided")
                    statusList.append(self.isColliding(v1[i], ent1Index, ent2Index, norm))                
                

        return min(statusList)
    
    def checkSphereCollisions(self, ent1Index, ent2Index):
        """
        """
        status = NOCOLLISION
        ent1 = self.listOfEntities[ent1Index]
        ent2 = self.listOfEntities[ent2Index]
        dMag = np.linalg.norm(ent1.body.x.v - ent2.body.x.v)
        
#        absSeparation = abs(dMag - ent1.boundRadius - ent2.boundRadius)
        
#        if absSeparation < COLTOL:
        n = ent1.body.x.v - ent2.body.x.v
        nNorm = n/np.linalg.norm(n)
        pt1FromBody = -nNorm*ent1.boundRadius
        pt2FromBody = nNorm*ent2.boundRadius
        pt1 = ent1.body.x.v + pt1FromBody
        pt2 = ent2.body.x.v + pt2FromBody
        relVel = ent1.body.v.v - ent2.body.v.v
        tangent = -np.cross(ent1.body.omega.v, pt1FromBody) + np.cross(ent2.body.omega.v, pt2FromBody)
        
        Vrn = np.dot(relVel, nNorm)
        if Vrn < 0:
            
            status = COLLISION
            self.listOfCollisions.append(Collision(ent1Index, ent2Index, nNorm, pt1, relVel, tangent))
            
#        elif absSeparation > COLTOL:
##            print(absSeparation)
#            status = PENETRATING


            
        return status
    
    def checkWallCollisions(self, entIndex):
        """
        """
        statusList = [NOCOLLISION]
        ent = self.listOfEntities[entIndex]
        pos = ent.body.x
        verts = ent.vertices
        
        if self.r is not None:
            
            if pos.x + ent.boundRadius > self.r:
                                    
                u = np.array([0,1,0])
                v = np.array([0,0,1])
                p = np.array([self.r,0,0])
                
                for vert in verts:
                    d = self.calcDistanceFromPointToPlane(vert,u,v,p)
                    
                    if np.linalg.norm(d) < COLTOL:
                        statusList.append(self.isCollidingWithWall(vert, entIndex, -5, u, v))
            
        if self.l is not None:
            
            if pos.x - ent.boundRadius < self.l:
                u = np.array([0,1,0])
                v = np.array([0,0,1])
                p = np.array([self.l,0,0])
                
                for vert in verts:
                    d = self.calcDistanceFromPointToPlane(vert,u,v,p)
                    
                    if np.linalg.norm(d) < COLTOL:
                        statusList.append(self.isCollidingWithWall(vert, entIndex, -6, u, v))
            
        if self.t is not None:
            
            if pos.y + ent.boundRadius > self.t:
                u = np.array([0,1,0])
                v = np.array([1,0,0])
                p = np.array([0,self.t,0])
                
                for vert in verts:
                    d = self.calcDistanceFromPointToPlane(vert,u,v,p)
                    
                    if np.linalg.norm(d) < COLTOL:
                        statusList.append(self.isCollidingWithWall(vert, entIndex, -4, u, v))
           
        if self.b is not None:
            
            if pos.y - ent.boundRadius < self.b:
                u = np.array([0,1,0])
                v = np.array([1,0,0])
                p = np.array([0,self.b,0])
                
                for vert in verts:
                    d = self.calcDistanceFromPointToPlane(vert,u,v,p)
                    
                    if np.linalg.norm(d) < COLTOL:
                        statusList.append(self.isCollidingWithWall(vert, entIndex, -3, u, v))
                        
        if self.f is not None:
            
            if pos.z + ent.boundRadius > self.f:
                u = np.array([0,0,1])
                v = np.array([1,0,0])
                p = np.array([0,0,self.f])
                
                for vert in verts:
                    d = self.calcDistanceFromPointToPlane(vert,u,v,p)
                    
                    if np.linalg.norm(d) < COLTOL:
                        statusList.append(self.isCollidingWithWall(vert, entIndex, -2, u, v))
                        
        if self.a is not None:
            
            if pos.z - ent.boundRadius < self.a:
                u = np.array([0,0,1])
                v = np.array([1,0,0])
                p = np.array([0,0,self.a])
                
                for vert in verts:
                    d = self.calcDistanceFromPointToPlane(vert,u,v,p)
                    
                    if np.linalg.norm(d) < COLTOL:
                        statusList.append(self.isCollidingWithWall(vert, entIndex, -1, u, v))
                        
        return min(statusList)
    
    def calcDistanceFromPointToPlane(self, pt, u, v, ptOnPlane):
        """
        """
        n = np.cross(u,v)
        PQ = pt - ptOnPlane
        
        n = n/np.linalg.norm(n)
        
        d = np.dot(PQ,n)
        
        # Used SymPy to check my answers, but this code runs attrociously slow
#        plane = Plane(ptOnPlane, normal_vector = np.cross(u, v))
#        d2 = float(plane.distance(Point3D(pt)))
        
        return d
    
    def isPointOnFace(self, pt, f):
        u = f[1] - f[0]
        v = f[3] - f[0]
        n = np.cross(u,v)
        status = False
        
        if abs(n[0]) > abs(n[1]) and abs(n[0]) > abs(n[2]):
            vList = [np.array([f[i][1], f[i][2], 0]) for i in range(4)] 
            p = np.array([pt[1], pt[2], 0])
            
            if self.pnpoly(4, vList, p) is True:
                status = True
            
        elif abs(n[1]) > abs(n[0]) and abs(n[1]) > abs(n[2]):
            vList = [np.array([f[i][0], f[i][2], 0]) for i in range(4)] 
            p = np.array([pt[0], pt[2], 0])
            
            if self.pnpoly(4, vList, p) is True:
                status = True
            
        elif abs(n[2]) > abs(n[0]) and abs(n[2]) > abs(n[1]):
            vList = [np.array([f[i][1], f[i][2], 0]) for i in range(4)] 
            p = np.array([pt[1], pt[2], 0])
            
            if self.pnpoly(4, vList, p) is True:
                status = True
        
        return status
    
    def pnpoly(self, npol, vList, p):
        """
        """
        poly = matplotlib.path.Path(np.array(vList).reshape([npol,3])[:,0:2])
        return poly.contains_point(p)
    
    def resolveCollisions(self):
        """
        """
        
        fCr = COEFRESTITUTION
        mu = COEFFRICTION
        
        for collision in self.listOfCollisions:
            ent1 = self.listOfEntities[collision.ent1Index]
            b1 = ent1.body
            
            if collision.ent2Index > 0: #If not a collision with a wall
                                
                ent2 = self.listOfEntities[collision.ent2Index]
                b2 = ent2.body

                pt1 = collision.point - b1.x.v
                pt2 = collision.point - b2.x.v
                
                j = ((-1 * (1 + fCr) * np.dot(collision.relVel, collision.normal)) / 
                    ((1/b1.mass + 1/b2.mass) + 
                     (np.dot(collision.normal,np.cross((np.cross(pt1,collision.normal) @ b1.IbodyInv),pt1))+
                      np.dot(collision.normal,np.cross((np.cross(pt2,collision.normal) @ b2.IbodyInv),pt2)))))
                    
                Vrt = np.cross(collision.relVel,collision.tangent)
                
                if np.linalg.norm(Vrt) > COLTOL:
                    b1Pchange =  ((j * collision.normal) + ((mu * j) * collision.tangent))# / b1.mass
                    b1Lchange =  (np.cross(pt1, ((j * collision.normal) + ((mu * j) * collision.tangent))))# @ b1.IbodyInv
                    
                    b2Pchange = -((j * collision.normal) + ((mu * j) * collision.tangent))# / b2.mass
                    b2Lchange = -(np.cross(pt2, ((j * collision.normal) + ((mu * j) * collision.tangent))))# @ b2.IbodyInv
                    
                else:
                    b1Pchange = (j * collision.normal)# / b1.mass
                    b1Lchange = (np.cross(pt1, (j * collision.normal)))# @ b1.IbodyInv
                    
                    b2Pchange = -(j * collision.normal)# / b2.mass
                    b2Lchange = -(np.cross(pt2, (j * collision.normal)))# @ b2.IbodyInv
                    

                b2.P += Vector.Quaternion(0,b2Pchange)
                b2.L += Vector.Quaternion(0,b2Lchange)

            else: #If colliding with a wall
#                print("collision with Wall")
                
                pt1 = collision.point - b1.x.v
                
                j = ((-1 * (1 + fCr) * np.dot(collision.relVel,collision.normal)) / 
                    ((1/b1.mass) +
                     (np.dot(collision.normal, np.cross(np.cross(pt1, collision.normal) @ b1.IbodyInv, pt1)))))
                
#                Vrt = np.cross(collision.relVel, collision.tangent)
                
#                if np.linalg.norm(Vrt) > COLTOL:
                b1Pchange = (j * collision.normal)# / b1.mass
                b1Lchange = np.cross(pt1, (j * collision.normal))# @ b1.IbodyInv
            
            b1.P += Vector.Quaternion(0,b1Pchange)
            b1.L += Vector.Quaternion(0,b1Lchange)
#            print("A collision Happened!")
            
class CollisionHandlerRepulsion(object):
    """ This class handles all collisions. Initialize and then call checkCollisions()
    every frame.
    """
    def __init__(self, listOfEntities, l,r,t,b,f,a,g):
        
        self.listOfEntities = listOfEntities
        self.l = l # left wall position
        self.r = r # right wall position
        self.t = t # top wall position
        self.b = b # bottom wall position
        self.f = f # fore wall position
        self.a = a # aft wall position
        
        self.g = g # gravity boolean
            
        self.physicsClient = pyb.connect(pyb.DIRECT)
        self.listOfCents = []
        
        for ent in self.listOfEntities:
            if ent.entityType is "box":
                cid = pyb.createCollisionShape(pyb.GEOM_BOX,
                    halfExtents=ent.size)
                cent = pyb.createMultiBody(baseCollisionShapeIndex = cid)
                pyb.resetBasePositionAndOrientation(cent, ent.body.pos.v, ent.body.q.asArrayForPyBullet())
                self.listOfCents.append(cent)
                
            if ent.entityType is "ball":
                cid = pyb.createCollisionShape(pyb.GEOM_SPHERE,
                                                     radius = ent.boundRadius)
                cent = pyb.createMultiBody(baseCollisionShapeIndex = cid)
                pyb.resetBasePositionAndOrientation(cent, ent.body.pos.v, ent.body.q.asArrayForPyBullet())
                self.listOfCents.append(cent)
            
        if self.l is not None:
            xSize = (self.r - self.l)/2
            ySize = (self.t - self.b)/2
            zSize = (self.f - self.a)/2
            cid = pyb.createCollisionShape(pyb.GEOM_BOX, halfExtents=[xSize,ySize,zSize])
            
            linkPos = [[xSize*4,0,0], [xSize*2,ySize*2,0], [xSize*2,-ySize*2,0],
                       [xSize*2,0,zSize*2], [xSize*2,0,-zSize*2]]

            self.wallBody = pyb.createMultiBody(1,cid,
                    linkMasses                    = [1.0]*5,
                    linkCollisionShapeIndices     = [cid]*5,
                    linkVisualShapeIndices        = [-1]*5,
                    linkPositions                 = linkPos, 
                    linkOrientations              = [[0,0,0,1]]*5,
                    linkInertialFramePositions    = [[0,0,0]]*5,
                    linkInertialFrameOrientations = [[0,0,0,1]]*5,
                    linkParentIndices             = [0]*5,
                    linkJointTypes                = [pyb.JOINT_FIXED]*5,
                    linkJointAxis                 = [[1,0,0]]*5
                    )
            pyb.resetBasePositionAndOrientation(self.wallBody, [-xSize*2,0,0], [0,0,0,1])
            
    def checkCollisions(self):
        """
        """
        self.listOfCollisions = []
        tryAgain = False
        statusList = [NOCOLLISION]
        cents = self.listOfCents
        self.collisionPairs = []
        
        for i, ent in enumerate(self.listOfEntities):
            pyb.resetBasePositionAndOrientation(cents[i], ent.body.x.v, ent.body.q.asArrayForPyBullet())

        for i, ent1 in enumerate(self.listOfEntities):
            
            # Check for collision between bodies
            for j, ent2 in enumerate(self.listOfEntities[i+1:]):
                k = i+j+1

                colPoints = pyb.getClosestPoints(cents[i],cents[k],COLTOL)
                if colPoints is not ():
                    for col in colPoints:
                        statusList.append(self.isColliding(i,k,col))  

            # Check for collision with walls
            if self.l is not None:
                colPointsWall = pyb.getClosestPoints(cents[i],self.wallBody,COLTOL)
                if colPointsWall is not ():
                    for col in colPointsWall:
                        statusList.append(self.isColliding(i,-1,col))
                
        if min(statusList) is COLLISION:
#            self.resolveCollisions()
            tryAgain = False
        elif min(statusList) is PENETRATING:
            tryAgain = True
#            self.resolveCollisions()
            
        return tryAgain
    
    def isColliding(self, ent1Index, ent2Index, colData):
        """
        """
#        if colData[8]<-COLTOL:
#            print(colData[8])
#            return PENETRATING
        
#        colData = colData[0]
#        contactFlag = colData[0]
#        idA = colData[1]
#        idB = colData[2]
#        linkIndex1 = colData[3]
#        linkIndex2 = colData[4]
        pt1 = np.array(colData[5])
        pt2 = np.array(colData[6])
        dist = colData[8]
#        normalForce = colData[9]
        
        status = NOCOLLISION
        ent1 = self.listOfEntities[ent1Index]
        
        pt1FromBody = pt1-ent1.body.x.v
        pt1 = ent1.body.x.v + pt1FromBody
        
        #If the other thing is not a wall
        if ent2Index != -1:
            ent2 = self.listOfEntities[ent2Index]
        
            if ent1.entityType is 'box' and ent2.entityType is 'box':
                n = pt1-pt2
                if dist < 0:
                    n *= -1
                
            else: 
                n = colData[7] # Check whether this is actually what I want
                
            pt2FromBody = pt2-ent2.body.x.v
#            relVel = ent1.body.v.v - ent2.body.v.v
#            tangent = -np.cross(ent1.body.omega.v, pt1FromBody) + np.cross(ent2.body.omega.v, pt2FromBody)
            
        #If the other thing is a wall
        else:
            if ent1.entityType is 'box':
                n = pt1-pt2
                if dist < 0:
                    n *= -1
                
            else: 
                n = colData[7] # Check whether this is actually what I want
                
#            tangent = -np.cross(ent1.body.omega.v, pt1FromBody)
#            relVel = ent1.body.v.v
        
        nNorm = n/np.linalg.norm(n)
#        print(nNorm)

#        pt2 = ent2.body.x.v + pt2FromBody
        
#        Vrn = np.dot(relVel, nNorm)
#        if Vrn < 0:
            
#        status = COLLISION
#        self.listOfCollisions.append(Collision(ent1Index, ent2Index, nNorm, pt1, relVel, tangent, dist))
        
        epsilon = 100
        rm = .1
        threshold = .00001
        power = 2
        k=1
        
#        if dist<0: dist=np.exp(dist)
#        r=COLTOL*np.exp(2*dist)
        if dist<threshold:
            r=threshold
#            print("too close")
        elif dist>=threshold: r=dist
#        print(r)
        

#        self.listOfEntities[ent1Index].body.torque += Vector.Quaternion.fromArray(np.cross(pt1FromBody, repulsiveForce))
        if ent2Index != -1: 
            repulsiveForce = nNorm*(self.LJF(r,rm,epsilon,power))
#        print(dist)
            self.listOfEntities[ent1Index].body.force += Vector.Quaternion.fromArray(repulsiveForce)
            self.listOfEntities[ent2Index].body.force += Vector.Quaternion.fromArray(-repulsiveForce)
#            self.listOfEntities[ent2Index].body.torque += Vector.Quaternion.fromArray(-np.cross(pt2FromBody,repulsiveForce))
            
#            if dist<-COLTOL:
#                status = PENETRATING
            
#            if dist < 0:
#                repulsiveForce  = Vector.Quaternion(0,nNorm*dist)*1000
#                ent1.body.force = repulsiveForce
#                ent2.body.force = -1*repulsiveForce
        else:
            if r<.2:
                repulsiveForce = nNorm*(k/r)
                self.listOfEntities[ent1Index].body.force += Vector.Quaternion.fromArray(repulsiveForce)

                
        return status
    
    @staticmethod
    def LJF(r, rm, epsilon,power):
        return power*epsilon*(rm**power/r**(power + 1) - rm**(power/5)/r**(power/5 + 1))
    
    @staticmethod
    def LJR(r, rm, epsilon):
        power = 12
        return epsilon*(rm/(r))**power
        
class Collision(object):
    def __init__(self, ent1Index, ent2Index, normal, point, relVel, tangent, dist=None):
        self.ent1Index = ent1Index
        self.ent2Index = ent2Index
        self.normal = normal
        self.point = point
        self.relVel = relVel
        self.tangent = tangent
        self.dist = dist