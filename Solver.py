# -*- coding: utf-8 -*-
"""
Created on Thu Feb 18 14:51:17 2016

@author: John
"""
from scipy.integrate import ode
from scipy.integrate import RK45
import numpy as np

#class Euler(object):
#    """A solver that uses Euler's method.
#
#    Attributes
#    -----
#    stepsize : float
#        The size between steps for Euler's method.
#    diffEq : class method
#        A method containing information about a differential equation.
#    """
#    def __init__(self, stepsize,diffEq):
#        self.stepsize = stepsize
#        self.diffEq = diffEq
#        
#    def advance(self,F,x):
#        """Advances the simulation based on Euler's method one step forward.
#
#        Parameters
#        -----
#        F : float
#            The current y value used in the differential equation.
#        x : float
#            The current x value used in the differential equation.
#
#        Returns
#        -----
#        FNext : float
#            The next y value after using Euler's method.
#        xNext : float
#            The next x value after one step.
#        """
#        FNext = F + self.diffEq(F,x)*self.stepsize
#        return FNext, x+self.stepsize
        
#class RK2(object):
#    """A solver that uses second order Runge-Kutta (RK2).
#
#    Attributes
#    -----
#    stepsize : float
#        The size between steps for RK2.
#    diffEq : class method
#        A method containing information about a differential equation.
#    """
#    def __init__(self, stepsize, diffEq):
#        self.stepsize = stepsize
#        self.diffEq = diffEq
#        
#    def advance(self,F,x):
#        """Advances the simulation based on a second order Runge-Kutta (RK2)
#        one step forward.
#
#        Parameters
#        -----
#        F : float
#            The current y value used in the differential equation.
#        x : float
#            The current x value used in the differential equation.
#
#        Returns
#        -----
#        FNext : float
#            The next y value after using RK2.
#        xNext : float
#            The next x value after one step.
#        """
#        step =  self.stepsize
#        k1 = self.diffEq(F,x)
#        k2 = step*self.diffEq(F + k1*step *0.5 , x + 0.5*step)
#        FNext = F + self.diffEq(F + self.diffEq(F,x)*self.stepsize *0.5,x)*self.stepsize 
#        return FNext, x+self.stepsize
        
class RK4(object):
    """A solver that uses fourth order Runge-Kutta (RK4).

    Attributes
    -----
    stepsize : float
        The size between steps for RK4.
    diffEq : class method
        A method containing information about a differential equation.
    """
    def __init__(self, diffEq):
#        self.stepSize = stepSize
        self.diffEq = diffEq
        
    def advance(self, x, F):
        """Advances the simulation based on a fourth order Runge-Kutta (RK4)
        one step forward.

        Parameters
        -----
        F : float
            The current y value used in the differential equation.
        x : float
            The current x value used in the differential equation.

        Returns
        -----
        FNext : float
            The next y value after using RK4.
        xNext : float
            The next x value after one step.
        """
#        dt  = self.stepSize
        dt = x #This isn't really ideal, but it'll work.
        
        k1    = dt * self.diffEq(x,          F)
        k2    = dt * self.diffEq(x + 0.5*dt, F + k1*0.5)
        k3    = dt * self.diffEq(x + 0.5*dt, F + k2*0.5)
        k4    = dt * self.diffEq(x + dt,     F + k3)
        
        FNext = F + (1/6.) * k1 + (1/3.) * k2 + (1/3.) * k3 + (1/6.) * k4
        
        return dt, FNext 
        
class RK4_SciPy(object):
    """Runge-Kutta differential equation solver from SciPy

    Uses an adaptive RK4 routine.
    
    This has been replaced by RK45_SciPy

    Attributes
    ----------
    dx : float
        The step size with which to advance the solution

    tolerance : float, optional
        Tolerance of the solver at each step.
    """
    
    def __init__(self,diffEq):

        self.r = ode(diffEq)
        self.r.set_integrator('dopri5') 

    def advance(self,t,f):
        """ Advance the differential equations one step

        Parameters
        ----------
        f : float or array, float
            The dependent variable

        x : float
            The independent variable

        Returns
        -------
        fnext : float
            The value of the dependent variable at the next step

        xnext : float
            The value of the independent variable at the next step
        """
        
        # Invoke the SciPy solver.
        if t == 0: 
            self.r.set_initial_value(f)
            return t, f
        
        if t is not 0: 
            self.r.integrate(t)

            return self.r.t,self.r.y

class RK45_SciPy(object):
    """Runge-Kutta differential equation solver from SciPy

    Uses an adaptive RK4 routine plus some fancy RK5 error approximation
    or something.
    
    Parameters
    ----------
    diffEq : function
        The differential equation used in advancing the system

    tolerance : float (percentage), optional
        Relative tolerance of the solver at each step.
    """
    def __init__(self, diffEq, relTol = 0.000000001):
        self.diffEq = diffEq
        self.initialized = False
        self.tolerance = relTol

    def advance(self, dt, f):
        """Runge-Kutta differential equation solver advance function
    
        Uses an adaptive RK4 routine plus some fancy RK5 error approximation
        or something.
        
        This is currently only set up to work with relative time steps rather than
        absolute times since the simulation began.
        
        Parameters
        ----------
        dt : float
            The amount of time by which to advance the system. Can be positive or
            negative.
            
        f : 1D numpy array
            Dependent variables
        """
        if not self.initialized: 
            self.r = RK45(self.diffEq, 0, f, dt, rtol = self.tolerance, vectorized=True)
            self.initialized = True
        
        else: 
            self.r.t = 0
            self.r.t_bound = dt
            self.r.status = 'running'
            
#        i=0
        
        while self.r.status is not 'finished':
            self.r.step()
#            i+=1
#        self.r.status = 'finished'
            
        return self.r.t,self.r.y
    
class RK45_SciPy2(object):
    """I would not recommend using this one
    """
    
    def __init__(self, diffEq):
        self.diffEq = diffEq
        self.last_t = 0
        self.initialized = False
        
    def advance(self,t,f):
            
        r = RK45(self.diffEq,self.last_t,f,t,vectorized = True)
        self.last_t = t
        while r.status is not 'finished':
            r.step()
            
        return r.t,r.y