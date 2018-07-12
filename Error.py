# -*- coding: utf-8 -*-
"""
Created on Thu Mar 17 13:52:23 2016

@author: John
"""
#import Simulation
import numpy as np
import copy as cp
import matplotlib.pyplot as plt
import Physics
import Solver
import math
#import StopConditions

def vErrorSetup(refBody0, solver, stopCondition, M, desiredRadius, desiredOrbits):
    """Sets up an error function for velocity which uses a simulation and
    properties based on your inputs.

    Paramters
    -----
    refBody0 : instance of gravBody
        The reference body whose error will be calculated.
    solver : instance of solver
        The solver being used in the simulation.
    stopCondition : function from StopConditions
        The stopcondition used in thesimulation. One that terminates at half
        period is stronly recommended.
    M : float
        The mass of the central body used in the simulation.
    desiredRadius : float
        The goal radius; the difference between this number and the radius at
        half period of the simulation is the error being returned.
    desiredOrbits : int
        The desired number of orbits for use in the simulation. Defunct for now.

    Returns
    -----
    vError : function
        The velocity error function.
    """
    def vError(v):
        """The error function used in a Search method for finding an initial
        velocity that gives a trajectory that reaches the desired height.

        Parameters
        -----
        v : float
            The velocity whose error is being calculated.

        Returns
        -----
        rError : float
            The difference between the calculated height and the desired height.
        """
        
        print( '\n')
        print( 'Velocity being tested ' + repr(v))
        
        # Make a deep copy so the original body isn't ever overwritten.
        refBody=cp.deepcopy(refBody0)
        
        #Assign the body's new velocity
        refBody.velocity.y=v
                                    
        #Keep track of the step size.
        oldStep=cp.deepcopy(solver.stepsize)
        
        #Simulate
        errorOrbit = Simulation.OrbitSim(solver,[refBody],stopCondition,
                                    M,desiredOrbits)
        t,bodies = errorOrbit.simulateOrbit()

        
        # Calculate semimajor axis and period
        a=(refBody0.position.r+desiredRadius)/2
        p=math.sqrt(a**3*4*math.pi**2/(M*6.7408*10**-11))
        
        #This is a variable only because I wasn't sure i the second to last
        #body was always going to be before the half period point. It is.
        index = -2
        
        #In order to find out how far the velocity ACTUALLY is, we want to step
        #directly to p/2. The next block takes a step back in the previous
        #simulation and then steps directly to p/2 and resimulates the last
        #step.
        newStart=t[index]
        step=p/2.-newStart
        solver.stepsize=step 
        newStartBody = bodies[0][index] #There is only one body, but it is
                                        #returned as a list.
        finalErrorOrbit = Simulation.OrbitSim(solver,[newStartBody],
                                    StopConditions.numStepsSetup(2,10000),
                                             M,desiredOrbits)
        time,bodies=finalErrorOrbit.simulateOrbit()
        
        #The last step we took directly to p/2 is the only one we care about.
        rGuess=bodies[0][-1].position.r
        print( 'rGuess = ' + repr(rGuess))
        
        #Calculate the error, make sure the step size is made normal again.
        rError=rGuess-desiredRadius
        solver.stepsize=oldStep

        return rError
    return vError
    
def tErrorSetup(v,refTime0,refBodies0,solver,stopCondition,M,desiredOrbits):
    """Sets up an error function for timewhich uses a simulation and properties
    based on your inputs. NOTE that it is assumed that the first body is the
    body whose parameters are being altered and the last body is the body you
    are trying to reach.

    Paramters
    -----
    v : float
        The launch velocity calculated ahead of time to reach the desired
        position's radius.
    refTime0 : list of floats
        The list of times corresponding to each frame of the refBodies0.
    refBody0 : list of lists of instances of gravBody
        The reference bodies whose data is used for more efficient simulations.
        Structured as [[list of body0's frames],[list of body1's frames]...]
    solver : instance of solver
        The solver being used in the simulation.
    stopCondition : function from StopConditions
        The stopcondition used in thesimulation. One that terminates at half
        period is stronly recommended.
    M : float
        The mass of the central body used in the simulation.
    desiredOrbits : int
        The desired number of orbits for use in the simulation. Defunct for now.

    Returns
    -----
    tError : function
        The time error function.
    """
    def tError(t):
        """The error function used in a Search method for finding an initial
        launch time that gives a trajectory that reaches the desired position
        at the desired time.

        Parameters
        -----
        v : float
            The velocity whose error is being calculated.

        Returns
        -----
        tError : float
            The distance between the search body's position at half period and
            the desired body's position.
        """
        
        print( '\n')
        print( 'Time being tested: ' + repr(t))

        #Make copies of the original information so it isn't overwritten.
        refTime,refBodies=cp.deepcopy(refTime0),cp.deepcopy(refBodies0)
        originalStep=cp.deepcopy(solver.stepsize)
        
        #We want to start at the time given, so we need to step to that point
        #using the closest point the reference paramters give us.
        priorStart=t-t%solver.stepsize #Makes t divisible by stepsize.
        priorStartPos=refTime.index(priorStart) #Position of start in the array
        
        #Grab all the reference bodies at the closest time to t.
        refBodies=np.array(refBodies)
        priorStartBodies=refBodies[:,priorStartPos]   
        
        #Calculate the sep needed to get to t and assign it.
        step=t-priorStart
        solver.stepsize=step
        
        #Set up the simulation and hop to the desired spot.
        stepToTime = Simulation.OrbitSim(solver,priorStartBodies,
                            StopConditions.numStepsSetup(2,3),M,desiredOrbits)
        newRefTime,newRefBodies=stepToTime.simulateOrbit()
    
        #Now our new starting spot is the last entry of the jump we just simulated.
        newRefBodies=np.array(newRefBodies)
        newStartBodies = newRefBodies[:,-1]
        
        #Reassign the step size
        solver.stepsize=originalStep    
    
        #Identify the body being launched and change its velocity.
        errorBody = newStartBodies[0]
        errorBody.velocity.r = v
        newStartBodies[0]=errorBody
        
        #Set up orbit and simulate to the closest we can get to p/2.
        newErrorOrbit = Simulation.OrbitSim(solver,newStartBodies,stopCondition,
                                    M,desiredOrbits)
        time,bodies=newErrorOrbit.simulateOrbit()
        
        #Graph for Assignment 6.1
#        zShip=[g.position.z for g in bodies[0]]
#        yShip=[g.position.y for g in bodies[0]]
#        xShip=[g.position.x for g in bodies[0]]
#        
#        zEarth=[g.position.z for g in bodies[1]]
#        yEarth=[g.position.y for g in bodies[1]]
#        xEarth=[g.position.x for g in bodies[1]]
#        
#        zMars=[g.position.z for g in bodies[2]]
#        yMars=[g.position.y for g in bodies[2]]
#        xMars=[g.position.x for g in bodies[2]]
#        
#        plt.plot(xEarth,yEarth,'.')
#        plt.plot(xShip,yShip,'.')
#        plt.plot(xMars,yMars,'.')
#        plt.plot(0,0, 'y*')
        
        #Calculate the semimajor axis and period.
        a=((refBodies0[1][0].position.r+refBodies0[2][0].position.r)/2.)
        p=math.sqrt(a**3*4*math.pi**2/(M*6.7408*10**-11))
        
        #Go to the second to last time frame
        index = -2
        newStart=time[index]
        newStartBodies = [bodies[0][index],bodies[1][index],bodies[2][index]]
        
        #Calculate the step needed to get to exactly p/2.
        step=p/2.-newStart
        solver.stepsize=step 

        #Simulate the last step.
        finalErrorOrbit = Simulation.OrbitSim(solver,newStartBodies,
                                        StopConditions.numStepsSetup(2,10000),
                                        M,desiredOrbits)
        time,bodies=finalErrorOrbit.simulateOrbit()      
        
        #Get the data from the last step.
        posBod1=bodies[0][-1].position
        posBod2=bodies[-1][-1].position
        
        #Reassign the step size
        solver.stepsize=originalStep 

        #Calculate error to be returned.
        posError=posBod1-posBod2

        return posError.r
    return tError
    
def lastStep(v,t,refTime0,refBodies0,solver,stopCondition,M,desiredOrbits):
    """Given a known launch velocity and time, this will tell you the final
    error from the desired position at the desired time.

    Paramters
    -----
    v : float
        The launch velocity calculated ahead of time to reach the desired
        position's radius.
    t : float
        The launch time calculated ahead of time to reach the desired position
        at the desired time.
    refTime0 : list of floats
        The list of times corresponding to each frame of the refBodies0.
    refBody0 : list of lists of instances of gravBody
        The reference bodies whose data is used for more efficient simulations.
        Structured as [[list of body0's frames],[list of body1's frames]...]
    solver : instance of solver
        The solver being used in the simulation.
    stopCondition : function from StopConditions
        The stopcondition used in thesimulation. One that terminates at half
        period is stronly recommended.
    M : float
        The mass of the central body used in the simulation.
    desiredOrbits : int
        The desired number of orbits for use in the simulation. Defunct for now.

    Returns
    -----
    error : float
        The error in the final position of the two bodies.
    """
    
    #Make copies of original data so it isn't overwritten.
    refTime,refBodies=cp.deepcopy(refTime0),cp.deepcopy(refBodies0)
    originalStep=cp.deepcopy(solver.stepsize)    
    
    #Calculate the start time that lines up with the reference bodies.
    priorStart=t-t%solver.stepsize
    priorStartPos=refTime.index(priorStart)
    
    #Select all of the bodies at that time.
    refBodies=np.array(refBodies)
    priorStartBodies=refBodies[:,priorStartPos]   
    
    #Calculate and assign stepsize to get to the desired time.
    step=t-priorStart
    solver.stepsize=step
    
    #Step to that time.
    stepToTime = Simulation.OrbitSim(solver,priorStartBodies,StopConditions.numStepsSetup(2,3),M,desiredOrbits)
    newRefTime,newRefBodies=stepToTime.simulateOrbit()
    
    #Select the data at the desired time
    newRefBodies=np.array(newRefBodies)
    newStartBodies = newRefBodies[:,-1]
    
    #Reassign step size for simulation over the half period
    solver.stepsize=originalStep    

    #Set up and simulate to the step after the half period
    errorBody = newStartBodies[0]
    errorBody.velocity.r = v
    newStartBodies[0]=errorBody
    newErrorOrbit = Simulation.OrbitSim(solver,newStartBodies,stopCondition,
                                M,desiredOrbits)
    time,bodies=newErrorOrbit.simulateOrbit()
    
    #Plot for 6.1
    zShip=[g.position.z for g in bodies[0]]
    yShip=[g.position.y for g in bodies[0]]
    xShip=[g.position.x for g in bodies[0]]
    
    zEarth=[g.position.z for g in bodies[1]]
    yEarth=[g.position.y for g in bodies[1]]
    xEarth=[g.position.x for g in bodies[1]]
    
    zMars=[g.position.z for g in bodies[2]]
    yMars=[g.position.y for g in bodies[2]]
    xMars=[g.position.x for g in bodies[2]]
    
    plt.plot(xEarth[0],yEarth[0],'wo')
    plt.plot(xMars[0],yMars[0],'wo')
    
    plt.plot(xEarth[:-2],yEarth[:-2],'.')
    plt.plot(xShip[:-2],yShip[:-2],'.')
    plt.plot(xMars[:-2],yMars[:-2],'.')
    plt.plot(0,0, 'y*')
    plt.title('Hohmann Transfer! Yay!')
    plt.xlabel('meters from Sun')
    plt.ylabel('meters from Sun')

    #Calculate semimajor axis and period
    a=((refBodies0[1][0].position.r+refBodies0[2][0].position.r)/2.)
    p=math.sqrt(a**3*4*math.pi**2/(M*6.7408*10**-11))
    
    #Get the second to last data point from the last simulation
    index = -2
    newStart=time[index]
    newStartBodies = [bodies[0][index],bodies[1][index],bodies[2][index]]
    
    #Calculate the step needed to get to the half period.
    step=p/2.-newStart
    solver.stepsize=step 

    #Simulate to the half period
    finalErrorOrbit = Simulation.OrbitSim(solver,newStartBodies,StopConditions.numStepsSetup(2,10000),
                                         M,desiredOrbits)
    time,bodies=finalErrorOrbit.simulateOrbit()
    
    #Plot for assignment 6.1
    plt.plot([bodies[0][-1].position.x,bodies[-1][-1].position.x],[bodies[0][-1].position.y,bodies[-1][-1].position.y],'o')
        
    #Calculate the error in distance between the two bodies at the half period.
    posError=bodies[0][-1].position-bodies[-1][-1].position

    return posError.r
    
def dErrorSetup(solver):
    """
    """
    def dError(t):
        """
        """
        stop=5000
        stepsize=300000
        diffEQ=Physics.nBody.diffEq
        solver=Solver.RK4(stepsize,diffEQ)
        stopCondition=StopConditions.afterTimeSetup(stop,time=t)
        Ms=1.989*10**30
        Mp=1.989*10**29
        ap=1.5*10**11
        Rs=6.0*10**8
        Rp=Rs/2.
        e,i,omega=0.0,np.pi/2.,0
        nbodySim=Simulation.ExoSim(solver,stopCondition,Physics.nBody,Ms,Mp,Rs,
                                   Rp,ap,e,omega,i)
        time,bodies=nbodySim.simulateOrbit()
        d=np.sqrt((bodies[0][-1].position.x-bodies[1][-1].position.x)**2+
            (bodies[0][-1].position.y-bodies[1][-1].position.y)**2)
        
#        print( d)
        return d
    return dError

def penetrationErrorSetup(App, backupEntities, checkPenetration):
    """
    """
    def pError(t):
#        i+=1
#        if i>25: 
#            print('Timeout: Failed to resolve collision')
#            break
#        self.listOfEntities = cp.deepcopy(listOfEntities)
#        self.physics.listOfEntities = self.listOfEntities
#        self.colHandler.listOfEntities = self.listOfEntities
#        self.nextTime -= 0.1 #self.stepsize
#        print(self.nextTime)
        App.listOfEntities = backupEntities
        App.colHandler.listOfEntities = App.listOfEntities
        App.physics.listOfEntities = App.listOfEntities
        currentTime = App.physics.advance(t)
        nearestDistance = checkPenetration()
        print("Nearest Distance is " + str(nearestDistance))
        return nearestDistance
    
    return pError
        
        
