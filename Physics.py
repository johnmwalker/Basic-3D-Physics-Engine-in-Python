# -*- coding: utf-8 -*-
"""
Created on Thu Feb 18 14:50:12 2016

@author: John
"""
import numpy as np
import Body
from Vector import Quaternion as Q
    
class nBodyRotation(object):
    """
    """
    def __init__(self,listOfEntities):
        self.G = -1 #-6.67408*10**(-11) # m^3/(kgs^2), the Gravitational Constant
#        self.solver = solver  
#        solver.diffEq = self.diffEq
        self.listOfEntities = listOfEntities
#        self.stepSize = solver.stepSize
        
    def diffEq(self, t, F):
        """ Standard differential equations for motion.

        Parameters
        -----
        F : numpy array
            

        Returns
        -----
        
        """
        G = 1
        
        F = np.reshape(F,[-1,23])
        xArray = F[:,0:3]
        qArray = F[:,3:7]
        PArray = F[:,7:10]
        LArray = F[:,10:13]
        oArray = self.omega
        otherF = self.forces
        otherT = self.torques 
        masses = F[:,22]
                
        bodyDot = np.zeros(F.shape)
#        vArray = PArray/np.hstack((masses,masses,masses))
        n=F.shape[0]
        naxes=3


#        vArray    = np.empty([0,3]) # velocity
#        qDotArray = np.empty([0,4]) # change in q
#        FArray    = np.empty([0,3]) # Forces
#        TArray    = np.empty([0,3]) # Torques
        
        # Create an n x n x naxis cube from the position matrix
        # Broadcast backwards so that there are n copies of the position
        # matrix
        pcube   = xArray*np.ones((n,n,naxes))
        
        # Create an n x n x naxis cube of differences
        diff = xArray[:,None,:]-pcube  
        
        # Create an n x n matrix of distances        
        r = np.sum(diff**2,axis=2)**0.5
        
        # Create 1/r**3 matrix with ones on the diagonal
        invr3 = (r + np.identity(n))**-3

        # Put the denominator under the differences and attach the masses
        bm = diff*invr3[:,:,None]*masses[:,None,None]
        
        # Sum back 
        dvdt = np.sum(bm,axis=0)*G

        for j in range(len(masses)):

            vArray = PArray[j]/masses[j]

            FArray=masses[j]*dvdt[j]+otherF[j,:]
#            FArray=otherF[j,:]
            
            #Determine rotational orientation
            q = qArray[j,:]
            omega = oArray[j,:]
            if np.linalg.norm(omega) > 10**-7:
#                print(np.linalg.norm(omega))
                qDot = 0.5 * (Q.fromArray(omega)*Q.fromArray(q)) # global coordinates
                qDotArray = qDot.asArray()

#               qDot = 0.5 * (Q.fromArray(q)*Q.fromArray(omega)) # body coordinates
            else: qDotArray = np.array([1,0,0,0])

            TArray = otherT[j,:]
            
            bodyDot[j,0:3]=vArray
            bodyDot[j,3:7]=qDotArray
            bodyDot[j,7:10]=FArray
            bodyDot[j,10:13]=TArray
            bodyDot[j,13:]=np.zeros(10)
#            bodyDot[j,:] = np.hstack([vArray,qDotArray,FArray,TArray, np.zeros(10)]) 
            
        # Add a column of zeros on the front for shape compatibility and
        # returning to quaternion
#        FArray=np.hstack([np.zeros((FArray.shape[0],1)),FArray])
        
        #print(bodyDot[0,:13])

        return bodyDot.flatten()

    def advance(self, time):
        """Advances the given system forward by a step.

        Parameters
        -----
        time : float
            The time by which the system is being advanced.

        Returns
        -----
        nothing
        """
        
        ents = self.listOfEntities

        arrayForSolver = np.zeros([len(self.listOfEntities), ents[0].body.stateArrayLength]) 
        
        for i, ent in enumerate(ents):
            arrayForSolver[i,:] = ent.body.state
            self.omega = arrayForSolver[:,13:16]
            self.forces = arrayForSolver[:,16:19]
            self.torques = arrayForSolver[:,19:22]
            
        newTime, f = self.solver.advance(time, arrayForSolver.flatten())
        f = f.reshape([len(ents),-1])
        
        for i, ent in enumerate(ents):
            
            ent.body.state = f[i,:]
            ent.body.resetForceAndTorque()
                
#    def dxdt(self, t, x, xdot):
#        """
#        """
#        arrayToBodies(x)
#        for i in range(NBODIES):
#            computeForceAndTorque(t, bodies[i])
#            ddtStateToArray(bodies[i],xdot[i*stateSize])
#    
#    def ddtStateToArray(rb,xdot):
#        """
#        """
##        Rdot=star(rb.omega)@rb.R
##        xdot=np.array([rb.v,np.ndarray.flatten(Rdot),rb.force, rb.torque])
#        
#        xdot = qdot.asArray()
#        
#    @staticmethod
#    def star(a):
#        """
#        """
#        return np.array(([0,-a[2],a[1]],[a[2],0,-a[0]],[-a[1],a[0],0]))
#        
#        
#    @staticmethod
#    def arrayToBodies(x):
#        """
#        """
#        for i in range(NBODIES):
#            Body.RigidBody.arrayToState(bodies[i],x[i*stateSize])
#            
#    @staticmethod
#    def bodiesToArray(x):
#        """
#        """
#        for i in range(NBODIES):
#            Body.RigidBody.stateToArray(bodies[i],x[i*stateSize])
            
class newtonian(object):
    """
    """
    def __init__(self,listOfEntities):
        self.G = -1 
        self.listOfEntities = listOfEntities
        
    def diffEq(self, t, F):
        """ Standard differential equations for motion.

        Parameters
        -----
        F : numpy array

        Returns
        -----
        
        """        
        F = np.reshape(F,[-1,23])
        PArray = F[:,7:10]
        otherF = self.forces
        masses = F[:,22]
        drag = 0.1
                
        bodyDot = np.zeros(F.shape)

#        for j in range(len(masses)):

        vArray = PArray/masses.reshape([-1,1])
        vArray -= drag*vArray
#            speed = np.linalg.norm(vArray)
#            if speed>speedlimit: vArray = vArray/speed*speedlimit

#        FArray=otherF[j,:]
            
        bodyDot[:,0:3]=vArray
        bodyDot[:,7:10]=otherF


        return bodyDot.flatten()

    def advance(self, time):
        """Advances the given system forward by a step.

        Parameters
        -----
        time : float
            The time by which the system is being advanced. Can be negative.
            
        """
        
        ents = self.listOfEntities

        arrayForSolver = np.zeros([len(self.listOfEntities), ents[0].body.stateArrayLength]) 
        
        for i, ent in enumerate(ents):
            arrayForSolver[i,:] = ent.body.state
            self.forces = arrayForSolver[:,16:19]
            
        newTime, f = self.solver.advance(time, arrayForSolver.flatten())
        f = f.reshape([len(ents),-1])
        
        for i, ent in enumerate(ents):
            
            ent.body.state = f[i,:]
            ent.body.resetForceAndTorque()
