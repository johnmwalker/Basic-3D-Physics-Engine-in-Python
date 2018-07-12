# -*- coding: utf-8 -*-
"""
Created on Fri Aug 18 09:11:33 2017

@author: beoka
"""
import numpy as np
import App
import Vector
import Body
from PIL import Image
import struct
Q = Vector.Quaternion.fromArray
from pybullet import getQuaternionFromEuler as eToQ


class Level(object):
    """ Houses all the code necessary for building sprites, determining the 
    speed and location of bodies, etc. Basically builds the game world.
    """
    def __init__(self, imagesDirectory, level, ctx):
        self.level = level
        self.imagesDirectory = imagesDirectory
        self.ctx = ctx
        self.listOfEntities = []
        self.l, self.r, self.t, self.b, self.f, self.a, = [None]*6
        self.gravity = True
        self.box = False
        
        # This runs level#() without having to do a bunch of if statements
        getattr(self,'level'+str(level))()
        
    def getAttributes(self):
        
        return  self.listOfEntities, self.l, self.r, self.t, self.b, self.f, self.a, self.box, self.gravity

        
    def level1(self):
        """
        """
        self.level=1
        ##### Parameters for Autobuilder #####
    
        M         =2.*10**34
        smallM    =[2.*10**27, 7.*10**22]
        a         =[4.0*10**11, 3.0*10**10.3]
        e         =[0,0]
        imageString = [self.imagesDirectory + "ganymedeSmall.png",
                       self.imagesDirectory + "gasPlanetSmall.png"]
    
        ##### Parameters for Manual builder #####
    
        mass=[M, 1*10**23  , 1*10**24 , 1*10**25 , 1*10**26 ]
        xPos=[0, 1*10**11, -1*10**11, 2.*10**11, 3.*10**11]
        yPos=[0, 0,          0,         0,         0        ]
        zPos=[0, 0,          0,         0,         0        ]
    
        vel=self.orbVel(M,np.array(mass[1:]),np.array(xPos[1:]))
    
        xVel=[0, 0,          0,         0,         0        ]
        yVel=[0, vel[0],     -vel[1],    vel[2],    vel[3]   ]
        zVel=[0, 0,          0,         0,         0        ]
        imageString2 =[self.imagesDirectory + "blackhole.png",
                       self.imagesDirectory + "planetSmall.png",
                       self.imagesDirectory + "planet2Small.png",
                       self.imagesDirectory + "planet3Small.png",
                       self.imagesDirectory + "planet4Small.png"]
    
        ##### Build Rocket #####
    
        mRocket = [6.*10**30]
        xRocket,yRocket,zRocket    = [5.0*10**11], [0], [0]
        vxRocket,vyRocket,vzRocket = [0],[self.orbVel(M,mRocket[0],
                                      xRocket[0])],[0]
        rocketImage = [self.imagesDirectory + "shipSmall.png"]
        self.rocketImageOrig=pg.image.load(rocketImage[0])
    
        self.genSpriteAndBodManual(xRocket,yRocket,zRocket,
                                   vxRocket,vyRocket,vzRocket,
                                   mRocket,rocketImage)
        self.rocketOnImage=pg.image.load(self.imagesDirectory + "shipSmallThrusting.png")
        self.rocketOnImageOrig=pg.image.load(self.imagesDirectory + "shipSmallThrusting.png")
    
        ##### Initialize ship characteristics #####
        self.listOfSprites[0].angle = 0
        self.listOfSprites[0].thrust = 20000.
    
        ##### Build bodies #####
        self.genSpriteAndBodManual(xPos, yPos, zPos, xVel, yVel, zVel, mass,
                                                               imageString2)
    #        self.genSpriteAndBod(smallM,e,M,a,imageString,
    #                        moon=True,keepCentral=False)
    
        ##### Make central body a black hole and ship the 'main body' #####
        self.listOfSprites[1].blackHole = True
    
    def level2(self):
        """ Solar System!
        """
        self.level=2
        ##### Parameters for Manual builder #####
        mass=[1.989*10**30, 3.285*10**23, 4.867*10**24, 5.972*10**24, 6.39*10**23, 1.898*10**27, 5.683*10**26, 8.681*10**25, 1.024*10**26]
        xPos=[0,            57.9*10**9,   108.2*10**9,  149.6*10**9,  227.9*10**9, 778.3*10**9,  1427.0*10**9, 2871.0*10**9, 4497.1*10**9]
        yPos=[0,            0,            0,            0,            0,           0,            0,            0,            0,          ]
        zPos=[0,            0,            0,            0,            0,           0,            0,            0,            0,          ]
    
        vel=self.orbVel(1.989*10**30,np.array(mass[1:]),np.array(xPos[1:]))
    #        print vel
    
        xVel=[0,            0,            0,            0,            0,           0,            0,            0,            0,          ]
        yVel=[0,            vel[0],       vel[1],       vel[2],       vel[3],      vel[4],       vel[5],       vel[6],       vel[7]      ]
        zVel=[0,            0,            0,            0,            0,           0,            0,            0,            0,          ]
        imageString =[self.imagesDirectory + 'sun.png',self.imagesDirectory + 'mercurySmall.png',self.imagesDirectory + 'venusSmall.png',
                      self.imagesDirectory + 'earthSmall.png',self.imagesDirectory + 'marsSmall.png',self.imagesDirectory + 'jupiterSmall.png',
                      'saturn.png',self.imagesDirectory + 'uranusSmall.png',self.imagesDirectory + 'neptuneSmall.png']
    
        ##### Build Rocket #####
        mRocket = [2030.*10**3]
        xRocket,yRocket,zRocket    = [5.0*10**10], [0], [0]
        vxRocket,vyRocket,vzRocket = [0],[self.orbVel(1.989*10**30,mRocket[0],
                                      xRocket[0])],[0]
        rocketImage = [self.imagesDirectory + "shipSmall.png"]
        self.rocketImageOrig=pg.image.load(rocketImage[0])
    
        self.genSpriteAndBodManual(xRocket,yRocket,zRocket,
                                   vxRocket,vyRocket,vzRocket,
                                   mRocket,rocketImage)
        self.rocketOnImage=pg.image.load(self.imagesDirectory + "shipSmallThrusting.png")
        self.rocketOnImageOrig=pg.image.load(self.imagesDirectory + "shipSmallThrusting.png")
    
        ##### Initialize ship characteristics #####
        self.listOfSprites[0].angle= 0
        self.listOfSprites[0].thrust = 20000.
    
        ##### Build bodies #####
        self.genSpriteAndBodManual(xPos, yPos, zPos, xVel, yVel, zVel, mass,
                                                               imageString)
    
        ##### Make central body a black hole and ship the 'main body' #####
        self.listOfSprites[1].blackHole = True
    
    
    def level3(self):
        """ Random assortment of planets
        """
        self.level=3
        mass=[1*10**23,1.*10**22,1.*10**20,1.*10**20, 1.*10**22]
        xPos=[0,-5.*10**8,-2.*10**8,2.*10**8,5.*10**8]
        yPos=[0,0,0,2.*10**8,0]
        zPos=[0,0,0,0,0]
        xVel=[0,0,10.,0,0]
        yVel=[0,15.,10.,10.,-15.]
        zVel=[0,0,0,0,0]
    
        imageString = [self.imagesDirectory + 'jupiterSmall.png',self.imagesDirectory + 'uranusSmall.png',self.imagesDirectory + 'marsSmall.png',
                       self.imagesDirectory + 'venusSmall.png',self.imagesDirectory + 'neptuneSmall.png']
        self.genSpriteAndBodManual(xPos,yPos,zPos,xVel,yVel,zVel,mass,imageString)
        self.listOfSprites[0].mainBody=True
    
        self.f=-3.*10**8
        self.c=3.*10**8
        self.r=6.*10**8
        self.l=-6.*10**8
        self.box = True
    
    def level4(self):
        """ Earth in the middle, neptune and uranus come in on both sides
        """
        self.level=4
        mass=[8.681*10**25, 1.024*10**26,5.972*10**24]
        xPos=[-5.*10**8,5.*10**8,0]
        yPos=[0,0,0]
        zPos=[0,0,0]
        xVel=[1500.,-1500.,0]
        yVel=[0,0,0]
        zVel=[0,0,0]
    
        imageString = [self.imagesDirectory + 'uranusSmall.png',self.imagesDirectory + 'neptuneSmall.png',
                       self.imagesDirectory + 'earthSmall.png']
        self.genSpriteAndBodManual(xPos,yPos,zPos,xVel,yVel,zVel,mass,imageString)
        self.listOfSprites[0].mainBody=True
    
        self.f=-3.*10**8
        self.c=3.*10**8
        self.r=6.*10**8
        self.l=-6.*10**8
        self.box = True
    
    def level5(self):
        """ 3 balls equal in size and mass meet in the same frame in the center
        """
        self.level=5
        mass=[1.,1.,1.]
        xPos=[-1.,1.,0]
        yPos=[0,0,1.]
        zPos=[0,0,0]
        xVel=[0,0,0]
        yVel=[0,0,0]
        zVel=[0,0,0]
    
        imageString = [self.imagesDirectory + 'neptuneSmall.png',self.imagesDirectory + 'neptuneSmall.png',self.imagesDirectory + 'neptuneSmall.png']
        self.genSpriteAndBodManual(xPos,yPos,zPos,xVel,yVel,zVel,mass,imageString)
        self.listOfSprites[0].mainBody=True
    
        self.f=-3.7
        self.c=3.7
        self.r=7.
        self.l=-7.
    
    def level6(self):
        """ 2 boxes
        """
        self.level=6
        self.box = False
        self.gravity=True
        
        size = [[.1,.1,.1],[.1,.1,.1]]
        mass = [.2,.2]
        Ibody = [self.momentInertiaCube(size[i],mass[i]) for i in range(len(mass))]
        x=[np.array((0,0.3,0)), 
           np.array((0,-0.30,0))]
#        q=[Vector.Quaternion(1,np.array([0,0,0])).normalize(),
#           Vector.Quaternion(1,np.array([0,0,0])).normalize()]
        q=[Vector.Quaternion.fromPyBulletEuler(eToQ((np.pi/4, 0, np.pi/4))), Vector.Quaternion.fromPyBulletEuler(eToQ((0, 0, 0)))]
        P=[np.array((.0, -1, 0)),
           np.array((.0,1,0))]
        L=[np.array((0.0,0, 0.01)),
           np.array((0.05,.01,0))]
        
        images = [self.imagesDirectory + 'purmesh.jpg']

        
        for i in range(len(images)):
            img = Image.open(images[i]).convert('RGBA')
            tex = self.ctx.texture(img.size, 4, img.tobytes())
            tex.use(i)

        imageIndex = [0,0]
        entityTypes = ['box']*2
        
        self.genSpriteAndBodManualQ(mass,Ibody,x,q,P,L,imageIndex,size,entityTypes)
        
    def level7(self):
        """ A 4 body system of spheres
        """
        self.level=7
        self.box = True
        self.gravity=True
        
        self.setWalls([-1,1,1,-1,1,-1])
#        print('floor position is ' + str(self.f))
        
        size = [[.03]*3,[.03]*3,[.1]*3,[.05]*3]
        mass = [.2,.2,.1,.1]
        Ibody = [2/5*mass[i]*(size[i][0]**2+size[i][1]**2)*
                 np.array(([1,0,0],[0,1,0],[0,0,1])) for i in range(len(mass))]
        x=[np.array((.1, 0, 0)), np.array((-.1,0,0)), np.array((0,-.3,0)),np.array((.4,0,0))]
        q=[Vector.Quaternion.fromMatrix(np.array(([0,0,0],[0,0,0],[0,0,0])))]*len(mass)
        P=[np.array((0, .1, 0)), np.array((0,-.2,0)),np.array((.045,0,0)),np.array((0,0,0))]
        L=[np.array((.0, .0, .01)),np.array((0,0,.1)),np.array((0,0,-.1)),np.array((0,0,0))]
        
        images = [self.imagesDirectory + 'purmesh.jpg', self.imagesDirectory + 'wood.jpg']

        
        for i in range(len(images)):
            img = Image.open(images[i])
            tex = self.ctx.texture(img.size, 3, img.tobytes())
            tex.build_mipmaps()
            tex.use(i)

        imageIndex = [0,0,1,1]
        entityTypes = ['ball']*4
        
        self.genSpriteAndBodManualQ(mass,Ibody,x,q,P,L,imageIndex,size,entityTypes)
        
    def level8(self):
        """ A 3 body system 
        """
        self.level=8
        mass = [.1,.1,.5]
        Ibody = [m/12*np.array(([1,0,0],[0,1,0],[0,0,1])) for m in mass]
        x=[np.array((.1, 0, 0)), np.array((-.1,0,0)), np.array((0,-.5,0))]
        q=[Vector.Quaternion.fromMatrix(np.array(([0,0,0],[0,0,0],[0,0,0])))]*3
        P=[np.array((-.06, .0, 0)), np.array((0,-.01,0)),np.array((.019,0,0))]
        L=[np.array((.0, .0, .0)),np.array((0,0,-.4)),np.array((0,0,0))]
        
        images = [self.imagesDirectory + 'purmesh.jpg', self.imagesDirectory + 'crate.png']
        size = [[.03,.03,.03],[.04,.04,.04],[.1,.1,.1]]
        self.gravity=True
        
        for i in range(len(images)):
            img = Image.open(images[i]).convert('RGBA')
            tex = self.ctx.texture(img.size, 4, img.tobytes())
            tex.use(i)

        imageIndex = [1,1,0]
        entityTypes = ['box', 'box', 'ball']
        
        self.genSpriteAndBodManualQ(mass,Ibody,x,q,P,L,imageIndex,size,entityTypes)
        
    def level9(self):
        """ two planets along the y axis
        """
        self.level=9
        mass = [.1]*3
        Ibody = [mass[0]/12*np.array(([1,0,0],[0,1,0],[0,0,1]))]*3
        x=[np.array((0, -.2, 0)), np.array((0,.2,0)), np.array((0.2,0,0))]
        q=[Vector.Quaternion.fromMatrix(np.array(([0,0,0],[0,0,0],[0,0,0])))]*3
        P=[np.array((.0, .0, 0)), np.array((-.0,0,0)),np.array((0,0,0))]
        L=[np.array((.0, .0, 0)),np.array((0,0,5)),np.array((.0, .0, 0))]
        
        images = [self.imagesDirectory + 'purmesh.jpg', self.imagesDirectory+'wood.jpg']
        size = [[.1,.1,.1]]*3
        self.gravity=True
        
        for i in range(len(images)):
#            print( i)
            img = Image.open(images[i]).convert('RGBA')
            tex = self.ctx.texture(img.size, 4, img.tobytes())
            tex.use(i)

        imageIndex = [0,1,1]
        entityTypes = ['box']*3
        
        self.genSpriteAndBodManualQ(mass,Ibody,x,q,P,L,imageIndex,size,entityTypes)
        
    def level10(self):
        """ A bunch of small balls
        """
        self.level = 10
        self.box=True
        self.l, self.r, self.t, self.b, self.f, self.a = [-1, 1, 1, -1, 1, -1]
        n = 50
        mass = [1]*n
        r = 1/n
        Ibody = [self.momentInertiaSphere(mass[0],r)]*n
        x=[np.array([2*np.random.random()-1,
                     2*np.random.random()-1,
                     0
                     ]) for i in range(n)]
        q=[Vector.Quaternion.fromMatrix(np.array(([0,0,0],[0,0,0],[0,0,0])))]*n
        P=[np.array([np.random.random()*(-1)**i/10,
                     np.random.random()*(-1)**i/10,
                     0
                     ]) for i in range(n)]
        L=[np.array([0,0,0])]*n
        images = [self.imagesDirectory + 'purmesh.jpg']
        size = [np.array([r]*3)]*n
        self.gravity=True
        
        for i in range(len(images)):
            img = Image.open(images[i]).convert('RGBA')
            tex = self.ctx.texture(img.size, 4, img.tobytes())
            tex.use(i)
        
        imageIndex = [0]*n
        entityTypes = ['ball']*n
        
        self.genSpriteAndBodManualQ(mass,Ibody,x,q,P,L,imageIndex,size,entityTypes)
        
    def level11(self):
        """ A bunch of small cubes
        """
        self.level = 10
        self.box=True
        self.l, self.r, self.t, self.b, self.f, self.a = [-1, 1, 1, -1, 1, -1]
        n = 50
        mass = [1]*n
        r = 1/n
        Ibody = [self.momentInertiaSphere(mass[0],r)]*n
        x=[np.array([2*np.random.random()-1,
                     2*np.random.random()-1,
                     0
                     ]) for i in range(n)]
        q=[Vector.Quaternion.fromMatrix(np.array(([0,0,0],[0,0,0],[0,0,0])))]*n
        P=[np.array([np.random.random()*(-1)**i/10,
                     np.random.random()*(-1)**i/10,
                     0
                     ]) for i in range(n)]
        L=[np.array([0,0,0])]*n
        images = [self.imagesDirectory + 'crate.png']
        size = [np.array([r]*3)]*n
        self.gravity=True
        
        for i in range(len(images)):
            img = Image.open(images[i]).convert('RGBA')
            tex = self.ctx.texture(img.size, 4, img.tobytes())
            tex.use(i)
        
        imageIndex = [0]*n
        entityTypes = ['box']*n
        
        self.genSpriteAndBodManualQ(mass,Ibody,x,q,P,L,imageIndex,size,entityTypes)
        

    @staticmethod
    def orbVel(m1, m2, r):
        """ Automatically calculates orbital velocities needed to make a
        circular orbit around the central body. This velocity can easily be
        adjusted to get more elliptical orbits.
        
        Parameters
        -----
        m1 : float
            Mass of one body (usually the central mass).
        m2 : float
            Mass of the other body (usually the mass of the orbiting body).
        r : float
            The distance between the orbiting body and the center of mass.
        
        """
        r=abs(r)
        G=6.6408*10**-11
        return (G*(m1+m2)/r)**0.5
            
    def genSpriteAndBodManualQ(self, mass,Ibody,x,q,P,L, imageIndex,size,entityTypes):
        """ Generates new bodies and sprites with a manual set of parameters.

        Parameters
        -----
        xPos : list of float
            A list of all the x position values of the bodies being created.
        yPos : list of float
            A list of all the y position values of the bodies being created.
        zPos : list of float
            A list of all the z position values of the bodies being created.
        xPos : list of float
            A list of all the x velocity values of the bodies being created.
        yPos : list of float
            A list of all the y velocity values of the bodies being created.
        zPos : list of float
            A list of all the z velocity values of the bodies being created.
        mass : list of float
            A list of all the masses for each body being created
        imageString : list of strings
            A list of all the strings of the image file names for each body.
        """

        bodies = self.manualOrbitBuilderQ(mass,Ibody,x,q,P,L)
        for i in range(len(mass)):

            rectInfo=size[i]
            radius=rectInfo[0]
            bodies[i].radius=radius
            
            self.listOfEntities.append(App.SimEntity(bodies[i],imageIndex[i],entityTypes[i],rectInfo))

            
    @staticmethod
    def closed2BodyAutoBuilder(omega,i,mBig,mSmall,ap,e):
        """
        """
        G = 1 #6.67408*10**(-11)
        vp=np.sqrt(G*mBig**3*(1+e)/(ap*(mSmall+mBig)**2*(1-e)))
        vs=-(mSmall/mBig)*vp
        rp=ap-ap*e
        rs=-(mSmall/mBig)*rp
        
        body1x=rp
        body1y=0
        body1z=0
        body1pos=Vector.Vector(x=body1x,y=body1y,z=body1z)
        body1pos.rotZ(omega)
        body1pos.rotX(i)
        
        body2x=rs
        body2y=0
        body2z=0
        body2pos=Vector.Vector(x=body2x,y=body2y,z=body2z)
        body2pos.rotZ(omega)
        body2pos.rotX(i)
        
        body1vx=0
        body1vy=vp
        body1vz=0
        body1vel=Vector.Vector(x=body1vx,y=body1vy,z=body1vz)
        body1vel.rotZ(omega)
        body1vel.rotX(i)
        
        body2vx=0
        body2vy=vs
        body2vz=0
        body2vel=Vector.Vector(x=body2vx,y=body2vy,z=body2vz)
        body2vel.rotZ(omega)
        body2vel.rotX(i)
        
        body1=Body.GravBody(body1vel,body1pos,mSmall)
        body2=Body.GravBody(body2vel,body2pos,mBig)
        bodies=[body1,body2]
        
        return bodies
    
    @staticmethod
    def manualOrbitBuilderQ(mass, Ibody, x, q, P, L):
        """
        """
        
        bodies=[]
        
        for i in range(len(mass)):

            body    = Body.RigidBody(mass[i],Ibody[i],Q(x[i]),q[i],Q(P[i]),Q(L[i]))
            bodies.append(body)
            
        return bodies
    
    @staticmethod
    def momentInertiaCube(size,mass):
        mInertia = mass*size[0]**2/6*np.identity(3)
        return mInertia
    
    @staticmethod
    def momentInertiaRectPrism(size, mass):
        l = size[0]
        w = size[1]
        h = size[2]
        m = mass
        mInertia =  np.array([m/3*(l**2 + h**2), m/4*w*h, m/4*h*w,
                         m/4*w*l, m/3*(w**2 + h**2), m/4*h*l,
                         m/4*h*w, m/4*h*l, m/3*(l**2 + w**2)]).reshape([3,3])
        print(mInertia)
        return mInertia
    
    @staticmethod
    def momentInertiaSphere(size, mass):
        return 2/5*mass*size**2*np.identity(3)
    
    def setWalls(self,listOfWalls):
        self.l = listOfWalls[0]
        self.r = listOfWalls[1]
        self.t = listOfWalls[2]
        self.b = listOfWalls[3]
        self.f = listOfWalls[4]
        self.a = listOfWalls[5]
        