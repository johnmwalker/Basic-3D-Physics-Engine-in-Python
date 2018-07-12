# -*- coding: utf-8 -*-
"""
Created on Thu Feb 18 14:51:41 2016

@author: John
"""
import numpy as np
import Vector

#class ThermalBody(object):
#    """Creates a body object whose only attribute is a temperature.
#
#    Attributes
#    -----
#    temperature : float
#        The temperature of the body.
#    """
#    def __init__(self, temperature):
#        self.temperature = temperature
#        
#class GravBody(object):
#    """Creates a body object whose attributes are velocity and height.
#
#    Attributes
#    -----
#    velocity : instance of Vector
#        A vector containing the three velocity vectors, Vx, Vy, and Vz.
#    position : instance of Vector
#        A vector containing the three position vectors, x, y, and z (or height).
#    mass : float
#        Mass of body in kg.
#    """
#    def __init__(self,velocity,position,mass,radius=None):
#
#        self.position = position
#        self.velocity = velocity
#        self.mass = mass
#        self.radius = radius
#        
#    def serialize(self):
#        
#        vx = self.velocity.x
#        vy = self.velocity.y
#        vz = self.velocity.z
#        x = self.position.x
#        y = self.position.y
#        z = self.position.z
#        
#        return np.array([vx,vy,vz,x,y,z])
#        
#    def deserialize(self,array):
#        
#        self.velocity.x=array[0]
#        self.velocity.y=array[1]
#        self.velocity.z=array[2]
#        self.position.x=array[3]
#        self.position.y=array[4]
#        self.position.z=array[5]

class RigidBody(object):
    """ A body as outlined in the paper "Physically Based Modeling: Rigid Body 
    Simulation" by David Baraff at Pixar Animation Studios. An instance of
    RigidBody keeps track of all of the relevant parameters involved in the 
    simulation of a single body.
    
    Attributes
    -----
    mass : float
        The mass of the body.
    Ibody : numpy array
        The 3x3 inertia tensor of the body
    x : numpy array
        The 3D vector describing the position of the body in real space
    q : Instance of Vector.Quaternion
        The quaternion which describes the rotational orientation of the body.
    P : numpy array
        The 3D vector describing the linear momentum of the body in real space
    L : numpy array
        The 3D vector describing the angular momentum of the body in body frame
    """
    def __init__(self, mass, Ibody, x, q, P, L):
        
        self.stateSize = 23
        
        #Constants
        self.mass = mass
        self.Ibody = Ibody #Moment of Inertia Tensor in Body Frame
        self.IbodyInv=np.linalg.inv(Ibody)
        
        #State Variables
        self.x = x #Position
        self.q = q #Rotational unit quaternion
        self.P = P #Linear Momentum
        self.L = L #Rotational Momentum
        
#        #Derived Quantities    
#        self.R = RigidBody.quaternionToMatrix(q) #Rotational Orientation
#        self.Iinv = self.R @ self.IbodyInv @ self.R.T #Inertia Tensor in World Frame
#        self.I = self.R @ self.Ibody @ self.R.T
#        self.v = P/mass #Velocity
#        self.omega = Vector.Quaternion(0, self.Iinv @ L) #Rotational Velocity
        
        #Computed Quantities
        self.resetForceAndTorque()
        
        #For compatibility
#        self.position = Vector.Quaternion(0, x)
#        self.velocity = Vector.Quaternion(0, self.v)
        
    # Derivd Quantities
    @property
    def R(self): # Rotational Orientation matrix
        return self.q.asMatrix()
    @property
    def Iinv(self):# Inverse of Inertia Tensor in World Frame
        return self.R @ self.IbodyInv @ self.R.T 
    @property
    def I(self): # Inertia Tensor in World Frame
        return self.R @ self.Ibody @ self.R.T
    @property
    def v(self): # Velocity of body in World Frame
        return self.P/self.mass
    @property
    def velocity(self): # Velocity of body in World Frame
        return self.P/self.mass
    @property
    def pos(self):
        return self.x
    
    @property
    def omega(self): # Rotational Velocity of body in 
        return Vector.Quaternion(0, self.IbodyInv @ self.L.asArray()[1:])
    
    @R.setter
    def R(self, m): # Rotational Orientation matrix
        self.q = Vector.Quaternion.fromMatrix(m)
#    @Iinv.setter
#    def Iinv(self, m):# Inverse of Inertia Tensor in World Frame
#    @I.setter
#    def I(self,m): # Inertia Tensor in World Frame
    @v.setter
    def v(self,a): # Velocity of body in World Frame
        self.P = a*self.mass
    @velocity.setter
    def velocity(self,a): # Velocity of body in World Frame
        self.P = a*self.mass
    @omega.setter
    def omega(self,a): # Rotational Velocity of body in 
        self.L = Vector.Quaternion(0,self.I @ a.asArray()[1:])
    
    @property
    def state(self):
        """ Translates the parameters in a RigidBody relevant to differentiation
        to a single array.
        """
        return np.array([*self.x.asArray()[1:], # 3 [0:3]
                         *self.q.asArray(),     # 4 [3:7]
                         *self.P.asArray()[1:], # 3 [7:10]
                         *self.L.asArray()[1:], # 3 [10:13]
                         *self.omega.asArray()[1:],       # 3 [13:16]
                         *self.force.asArray()[1:],       # 3 [16:19]
                         *self.torque.asArray()[1:],      # 3 [19:22]
                          self.mass])           # 1 [-1]
    @property
    def stateArrayLength(self):
        return len(self.state)
    
    @state.setter
    def state(self,A):
        """ Translates an array consisting of the parameters of a RigidBody
        back to a RigidBody.
        """
        Q = Vector.Quaternion.fromArray
        self.x = Q(A[0:3])
        self.q = Q(A[3:7])
        self.q.normalize()
        self.P = Q(A[7:10])
        self.L = Q(A[10:13])
        
    def resetForceAndTorque(self):
        self.force = Vector.Quaternion.fromArray(np.zeros(3))
        self.torque = Vector.Quaternion.fromArray(np.zeros(3))
        
