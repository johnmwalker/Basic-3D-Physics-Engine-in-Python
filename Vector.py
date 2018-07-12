# -*- coding: utf-8 -*-
"""
Created on Tue Feb 16 13:56:08 2016

@author: John
"""
import math
import numpy as np

class Vector(object):
    """Generic Vector class in either two or three dimensions.

    So far supports basic boolean == or !=, addition, multiplication, and
    subtraction between Vector objects.

    Attributes
    -----
    x : float
        cartesian x component

    y : float
        cartesian y component

    r : float
        polar radial component

    theta : float
        polar azimuthal angle
    """
    
    def __init__(self, x=None,y=None,z=None,r=None,theta=None,phi=None):
        """ Let's get set up!"""
        if x is not None:
            self.x = x
            self.y = y
            self.z = z
        else:
            self.x = r*math.sin(phi)*math.cos(theta)
            self.y = r*math.sin(phi)*math.sin(theta)
            self.z = r*math.cos(phi)
            
    def rotX(self,theta):
        """
        """
        row1=np.array([1,0,0])
        row2=np.array([0,math.cos(theta),math.sin(theta)])
        row3=np.array([0,-math.sin(theta),math.cos(theta)])
        rotMatrix=np.vstack([row1,row2,row3])
        
        xyzMatrix=np.array([self.x,self.y,self.z])
        newxyzMatrix=np.dot(xyzMatrix,rotMatrix)
#        newxyzMatrix=np.sum(newxyzMatrix,axis=0)
        self.x=newxyzMatrix[0]
        self.y=newxyzMatrix[1]
        self.z=newxyzMatrix[2]
    
    def rotY(self,theta):
        """
        """
        row1=np.array([math.cos(theta),0,-math.sin(theta)])
        row2=np.array([0,1,0])
        row3=np.array([math.sin(theta),0,math.cos(theta)])
        rotMatrix=np.vstack([row1,row2,row3])
        
        xyzMatrix=np.array([self.x,self.y,self.z])
        newxyzMatrix=np.dot(xyzMatrix,rotMatrix)
#        newxyzMatrix=np.sum(newxyzMatrix,axis=0)
        self.x=newxyzMatrix[0]
        self.y=newxyzMatrix[1]
        self.z=newxyzMatrix[2]
        
    def rotZ(self,omega):
        """
        """
        row1=np.array([math.cos(omega),math.sin(omega),0])
        row2=np.array([-math.sin(omega),math.cos(omega),0])
        row3=np.array([0,0,1])
        rotMatrix=np.vstack([row1,row2,row3])
        
        xyzMatrix=np.array([self.x,self.y,self.z])
        newxyzMatrix=np.dot(xyzMatrix,rotMatrix)
#        newxyzMatrix=np.sum(newxyzMatrix,axis=0)
        self.x=newxyzMatrix[0]
        self.y=newxyzMatrix[1]
        self.z=newxyzMatrix[2]
      
    @property
    def r(self):
        return math.sqrt(self.x**2+self.y**2+self.z**2)
        
    @r.setter
    def r(self, r):
        x = self.x
        y = self.y   
        z = self.z
        theta = self.theta
        phi = self.phi
        
        self.x = r*math.sin(phi)*math.cos(theta)
        self.y = r*math.sin(phi)*math.sin(theta)
        self.z = r*math.cos(phi)
        
    @property
    def theta(self):
        theta=math.atan2(self.y, self.x)
        if theta < 0:
            theta = 2*math.pi+theta
        return theta
        
    @theta.setter
    def theta(self, theta):
        x = self.x
        y = self.y        
        r = self.r
        phi = self.phi
        
        self.x = r*math.sin(phi)*math.cos(theta)
        self.y = r*math.sin(phi)*math.sin(theta)
        
    @property
    def phi(self):
        return math.atan2(math.sqrt(self.x**2+self.y**2),self.z)
        
    @phi.setter
    def phi(self, phi):
        x = self.x
        y = self.y  
        z = self.z
        r = self.r
        theta = self.theta
        
        self.x = r*math.sin(phi)*math.cos(theta)
        self.y = r*math.sin(phi)*math.sin(theta)
        self.z = r*math.cos(phi)
        self.r = math.sqrt(x**2+y**2+z**2)
        
#    @property
#    def x(self):
#        return self.r*math.sin(self.phi)*math.cos(self.theta)
#        
#    @x.setter
#    def x(self, x):
#
#        r = self.r     
#        y = self.y
#        z = self.z
#        theta = self.theta
#        
#        self.r = math.sqrt(x**2 + y**2 + z**2)
#        self.theta = math.atan2(y, x)
#        
#    @property
#    def y(self):
#        return self.r*math.sin(self.phi)*math.sin(self.theta)
#        
#    @y.setter
#    def y(self, y):
#        r = self.r
#        x = self.x       
#        theta = self.theta
#        
#        self.r = math.sqrt(x**2 + y**2 + z**2)
#        self.theta = math.atan2(y, x)
#        
#    @property
#    def z(self):
#        return self.r*math.cos(self.phi)
#        
#    @z.setter
#    def z(self, z):
#        r = self.r
#        x = self.x     
#        y = self.y
#        theta = self.theta
#        
#        self.r = math.sqrt(x**2 + y**2 + z**2)
#        self.phi = math.atan2(math.sqrt(x**2+y**2),z)

        
    def __repr__(self):
        return "Vector(x=%f, y=%f, z=%f)"%(self.x,self.y,self.z)
        
    def __mul__(self,v):
        return Vector(x=self.x*v, y=self.y*v, z=self.z*v)

    def __add__(self,v):
        xNew = self.x+v.x
        yNew = self.y+v.y
        zNew = self.z+v.z
        return Vector(x=xNew, y=yNew, z=zNew)

    def __sub__(self,v):
        xNew = self.x-v.x
        yNew = self.y-v.y
        zNew = self.z-v.z
        return Vector(x=xNew, y=yNew, z=zNew)
    
#    def __lt__(self, other):
#        return self.pages < other
#    
#    def ___le__(self, other):
#        return self.pages <= other
    
    def __eq__(self, other):
        if self.x==other.x and self.y==other.y and self.z==other.z: 
            return True
        else:
            return False
    
    def __ne__(self, other):
        if self.x!=other.x or self.y!=other.y or self.z!=other.z:
            return True
        else:
            return False
    
#    def __gt__(self, other):
#        return self.pages > other
#    
#    def __ge__(self, other):
#        return self.pages >= other
    
    def __abs__(self):
        return Vector(x=abs(self.x),y=abs(self.y),z=abs(self.z))
        
    def asArray(self):
        return np.array([self.x,self.y,self.z])
        
class Quaternion(object):
    """ A class for managing quaternions.
    
        Attributes
        -----
        s : float
            The non-vector component of a quaternion
        v : np.array((x,y,z))
            A numpy array with the three vector components.
        
    """
    def __init__(self, s, v):
        
#        if type(s) != float or type(s) != int:
#            raise ValueError("s must be a number")
        if v.shape != (3,):
            raise ValueError("v must be a three dimensional numpy vector")
        self.s = s
        self.v = v
        
    def __mul__(self,other):
        
        if type(other)==Quaternion:
            
            s1=self.s
            s2=other.s
            v1=self.v
            v2=other.v
        
            q=Quaternion(s1*s2-np.dot(v1,v2),
                          s1*v2+s2*v1+np.cross(v1,v2))
            
        elif type(other)==int or type(other)==float:
            
            q=Quaternion(self.s*other,self.v*other)
            
        else: print("This type of multiplication is not supported")
        
        return q
    
    __rmul__ = __mul__
    
    def __div__(self,other):
        if type(other)==int or type(other)==float:
            q=Quaternion(self.s/other,self.v/other)
        else: print("This type of division is not supported")
        return q
    __truediv__ = __div__
    
    def __iadd__(self,other):
        self.s += other.s
        self.v += other.v
        
        return self
    
    def __add__(self,other):
        s1=self.s
        s2=other.s
        v1=self.v
        v2=other.v
        return Quaternion(s1+s2,v1+v2)
        
    def asArray(self):
        return np.array([self.s,self.v[0],self.v[1],self.v[2]])
    
    def asArrayForPyBullet(self):
        return np.array((self.v[0],self.v[1],self.v[2], self.s))
    
    @property
    def x(self):
        return self.v[0]
    @property
    def y(self):
        return self.v[1]
    @property
    def z(self):
        return self.v[2]
    
    @x.setter
    def x(self,x):
        self.v[0]=x
    @y.setter
    def y(self,y):
        self.v[1]=y
    @z.setter
    def z(self,z):
        self.v[2]=z
        
    @property
    def Xangle(self):
        """Returns the angle of orientation about the x axis in degrees.
        """
#        q0=self.s
#        q1=self.v[0]
#        q2=self.v[1]
#        q3=self.v[2]
#        angle = math.atan2(2*(q0*q1+q2*q3),1-2*(q1**2+q2**2))
#        angle = math.degrees(angle)
        
        ysqr = self.y**2
		
        t0 = +2.0 * (self.s * self.x + self.y*self.z)
        t1 = +1.0 - 2.0 * (self.x*self.x + ysqr)
        angle = math.degrees(math.atan2(t0, t1))

#        print(angle)
        return angle
    
    @property
    def Yangle(self):
        """Returns the angle of orientation about the y axis in degrees.
        """
#        q0=self.s
#        q1=self.v[0]
#        q2=self.v[1]
#        q3=self.v[2]
#        angle = math.asin(2*(q0*q2-q3*q1))
#        angle = math.degrees(angle)
#        
#        if abs(abs(self.lastAngle)-abs(angle)) > 90:
#            angle = angle + 180
#            self.lastAngle = angle
        		
#        t2 = +2.0 * (self.s*self.y - self.z*self.x)
#        t2 =  1 if t2 > 1 else t2
#        t2 = -1 if t2 < -1 else t2
#        angle = math.degrees(math.asin(t2))
        
        q1=self.v[0]
        q2=self.v[1]
        q3=self.v[2]
        q4=self.s
        angle = math.degrees(-math.cos(math.radians(self.psi))*2*(q1*q3-q2*q4)/(1-2*(q2**2+q3**2)))
        
#        print(angle)
        return angle
    
    @property
    def Zangle(self):
        """Returns the angle of orientation about the z axis in degrees.
        """
#        q0=self.s
#        q1=self.v[0]
#        q2=self.v[1]
#        q3=self.v[2]
#        angle = math.atan2(2*(q0*q3+q1*q2),1-2*(q2**2+q3**2))
#        angle = math.degrees(angle)
        
        ysqr = self.y**2
        t3 = +2.0 * (self.s * self.z + self.x*self.y)
        t4 = +1.0 - 2.0 * (ysqr + self.z**2)
        angle = math.degrees(math.atan2(t3, t4))
        
#        print(angle)
        return angle
    
    @property
    def eulerAngles(self):
        """Returns yaw, pitch, and roll in degrees.
        """
        n=self.s
        x=self.x
        y=self.y
        z=self.z
        
        r31 = 2*z*x - 2*y*n
        r32 = 2*z*y + 2*x*n
        r33 = n**2 - x**2 - y**2 + z**2
        r21 = 2*x*y + 2*z*n
        r11 = n**2 + x**2 - y**2 - z**2
        
        yaw = math.degrees(math.atan2(r21,r11))
        pitch = math.degrees(math.asin(-r31))
        roll = math.degrees(math.atan2(r32,r33))
        
        return [yaw, pitch, roll]
        
    def normalize(self):
        """Normalizes the quaternion to length of 1.
        """
        norm = np.linalg.norm(self.asArray())
        if norm>0.00001: 
            self.s = self.s/norm
            self.v = self.v/norm
        
        return Quaternion(self.s, self.v)
    
    def __repr__(self):
        return "Quaternion(s={}, x={}, y={}, z={})".format(self.s,self.v[0],self.v[1],self.v[2])
    
    @classmethod
    def fromArray(cls, a):
        """Turns a vector of length 4 into a quaternion where a[0] is the
        scalar and the last three components are the vector.
        """
        if len(a)==3:
            quat   = cls(0,a)
        else: quat = cls(a[0],a[1:])
        return quat
    
    def asMatrix(self):
        """ Translates a quaternion describing the rotational orientation of a
        RigidBody to a 3x3 matrix (numpy array) which represents the same.
        """
        
        n = self.s
        x = self.x
        y = self.y
        z = self.z
        
        r11 = n**2 + x**2 - y**2 - z**2
        r21 = 2*x*y + 2*z*n
        r31 = 2*z*x - 2*y*n
        
        r12 = 2*x*y - 2*z*n
        r22 = n**2 - x**2 + y**2 - z**2
        r32 = 2*z*y + 2*x*n
        
        r13 = 2*x*z + 2*y*n
        r23 = 2*y*z - 2*x*n        
        r33 = n**2 - x**2 - y**2 + z**2
        
        m = np.array([r11, r12, r13,
                      r21, r22, r23,
                      r31, r32, r33]).reshape([3,3])
        return m
    
    @classmethod
    def fromMatrix(cls, m):
        """ Translates a 3x3 matrix describing the rotational orientation of a
        RigidBody to a quaternion which represents the same.
        """
        
        tr = m[0,0] + m[1,1] + m[2,2]
        
        if tr >= 0:
            
            s=np.sqrt(tr + 1)
            a=0.5/s
            q=cls(0.5*s,np.array((
                         (m[2,1]-m[1,2])*a,
                         (m[0,2]-m[2,0])*a,
                         (m[1,0]-m[0,1])*a)))
            
        else:
            
            i=0
            if m[1,1] > m[0,0]:
                i=1
            if m[2,2] > m[i,i]:
                i=2
                
            if i==0:
                s=np.sqrt((m[0,0]-(m[1,1]+m[2,2]))+1)
                a=0.5/s
                q=cls((m[2,1]-m[1,2])*a,np.array((
                             (0.5*s,
                             (m[0,1]+m[1,0])*a,
                             (m[2,0]+m[0,2])*a))))
            elif i==1:
                s = np.sqrt((m[1,1]-(m[2,2]+m[0,0]))+1)
                a=0.5/a
                q=cls((m[0,2]-m[2,0])*a,np.array((
                             (m[0,1]+m[1,0])*a,
                             0.5*s,
                             (m[1,2]+m[2,1]*a))))
                
            elif i==2:
                s=np.sqrt((m[2,2]-(m[0,0]+m[1,1]))+1)
                a=0.5/s
                q=cls((m[1,0]-m[0,1])*a, np.array((
                             (m[2,0]+m[0,2])*a,
                             (m[1,2]+m[2,1])*a,
                             0.5*s)))
            else: print("Error!")
            
        return q
    
    @classmethod
    def fromPyBulletEuler(cls, a):
        """ Turns a 4 dimensional array from pybullet's getQuaternionFromEuler
        function into a Quaternion.
        """
        return cls(a[-1],np.array(a[0:3]))
            
#def main():
#    
#    print( '\n', "First tests are in two dimensions.", '\n')
#        
#    X = 1
#    Y = 2  
#    
#    V = Vector(x=X, y=Y)
#
####### Two-Dimensional Tests   
# 
#    if V.r != math.sqrt(X**2+Y**2):
#        print( 'You failed to calculate radius!'
#    else: print( 'Your radius is correct at ' + repr(V.r)
#        
#    if V.theta != math.atan2(Y,X):
#        print( 'You failed to calculate theta!'
#    else: print( 'Your theta is correct at ' + repr(V.theta)
#    
#    print( "Now we're going to reassign r and theta!"
#    
#    R = 1
#    Theta = math.pi    
#    
#    V.r = R
#    V.theta = Theta
#    
#    if V.x != math.cos(Theta)*R:
#        print( 'You failed to get x based on a new radius and theta!'
#    else: print( 'Your x value is correct at ' + repr(V.x)
#    
#    if V.y != math.sin(Theta)*R:
#        print( 'You failed to get a new y based on a new radius and theta!'
#    else: print( 'Your y value is correct at ' + repr(V.y)
    
###### Three-Dimensional Tests
#
#    print( '\n', "Next tests are in three dimensions.", '\n')
#
#    X = 1
#    Y = 2  
#    Z = 3
#    
#    V = Vector(x=X, y=Y, z=Z)
#    
#    if V.r != math.sqrt(X**2+Y**2+Z**2):
#        print( 'You failed to set radius based on x, y, and z!!!!!!')
#    else: print( 'Your radius is correct at ' + repr(V.r))
#        
#    if V.theta != math.atan2(Y,X):
#        print( 'You failed to set theta based on x, y, and z!!!!!!!')
#    else: print( 'Your theta is correct at ' + repr(V.theta))
#    
#    if V.phi != math.atan2(math.sqrt(X**2+Y**2),Z):
#        print( 'You failed to set phi based on x, y, and z!!!!!!!')
#    else: print( 'Your phi is correct at ' + repr(V.theta))
#    
#    print( '\n', "Now we're going to reassign r, theta, and phi!")
#    
#    R = 10
#    Theta = math.pi   
#    Phi = math.pi/2.
#    
#    V.r = R
#    V.theta = Theta
#    V.phi = Phi
#    
#    if V.x != math.sin(Phi)*math.cos(Theta)*R:
#        print( 'You failed to set a new x value based on a new radius and theta!!!!!')
#    else: print( 'Your x value is correct at ' + repr(V.x))
#    
#    if V.y != math.sin(Phi)*math.sin(Theta)*R:
#        print( 'You failed to set a new y value based on a new radius and theta!!!!!!')
#    else: print( 'Your y value is correct at ' + repr(V.y))
#    
#    if V.z != math.cos(Phi)*R:
#        print( 'You failed to set a new z value based on a new radius and theta!!!!!!')
#    else: print( 'Your z value is correct at ' + repr(V.y))
#       
#    print( '\n', "Now we're going to try some basic mathematical functions!")
#    
#    x1,y1,z1=(1,1,1)    
#    x2,y2,z2=(2,2,2)
#    
#    V1 = Vector(x=x1,y=y1,z=z1)
#    V2 = Vector(x=x2,y=y2,z=z2)
#    
#    if V1*V2 != x1*x2+y1*y2+z1*z2:
#        print( 'You failed to multiply successfully!')
#    else: print( 'Multiplication succeeded with the result ' + repr(V1*V2))
#    
#    if V1+V2 != Vector(x=x1+x2,y=y1+y2,z=z1+z2):
#        print( 'You failed to add successfully!')
#    else: print( 'Addition succeeded with the result ' + repr(V1+V2))
#    
#    if V1-V2 != Vector(x=x1-x2,y=y1-y2,z=z1-z2):
#        print( 'You failed to subtract successfully!')
#    else: print( 'Subtraction succeeded with the result ' + repr(V1-V2))
#
#    return V, V1, V2
#    
#if __name__ == '__main__': V, V1, V2 = main()