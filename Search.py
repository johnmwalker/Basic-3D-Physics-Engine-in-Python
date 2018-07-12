# -*- coding: utf-8 -*-
"""
Created on Thu Feb 25 15:06:26 2016

@author: John
"""
import numpy as np
import copy as cp

class Bisect(object):
    """Creates an instance for use in finding an independent variable based on
    a desired dependent variable. This class uses a modified bisection method.

    Attributes
    -----
    error : function
        This error function must take in an estimated independent variable and
        return an error in the y variable that is not absolute.
    xGuess : float
        An initial guess of the independent variable. Can be arbitrary, but the
        closer it is, the more quickly the algorithm works.
    tolerance : float
        The tolerance for how close the algorithm is able to get to the desired
        dependent variable. A value of zero will probably cause an infinite
        loop.

    """
    def __init__(self,error,xGuess,tolerance):
        self.error=error
        self.xGuess=xGuess
        self.tolerance=tolerance
        
    def find(self):
        """Finds an x value that gives a desired y value based on a given error
        function within a given tolerance. This method uses a modified
        bisection method.

        Parameters
        -----
        None

        Returns
        -----
        The estimated x value that gives the desired y value within a tolerance.
        """
        xGuess=self.xGuess
        lastxGuess = xGuess
        up = False
        down = False
        tolerance=self.tolerance
        error = self.error
        
        err=error(xGuess)

        while abs(err)>tolerance:

            print( 'err = ' + repr(err))
            if err < 0:
                up = True
                if not down:
                    lastxGuess = xGuess
                    xGuess = xGuess*2.
                else:
                    hold = xGuess
                    xGuess = xGuess + abs(lastxGuess-xGuess)*0.5
                    lastxGuess = hold
                    
            elif err > 0:
                down = True
                if not up:
                    lastxGuess = xGuess
                    xGuess = xGuess/2.
                else:
                    hold = xGuess
                    xGuess = xGuess - abs(xGuess-lastxGuess)*0.5
                    lastxGuess = hold
                    
            else: print( 'Error?')
            err = error(xGuess)
            
        return xGuess

    
class Newton(object):
    """Creates an instance for use in finding an independent variable based on
    a desired dependent variable. This class uses Newton's method.

    Attributes
    -----
    error : function
        This error function must take in an estimated independent variable and
        return an error in the y variable that is not absolute.
    xGuess : float
        An initial guess of the independent variable. Can be arbitrary, but the
        closer it is, the more quickly the algorithm works.
    tolerance : float
        The tolerance for how close the algorithm is able to get to the desired
        dependent variable. A value of zero will probably cause an infinite
        loop.
    delta : float
        For use in Newton's method, the smaller this is, the more accurate the
        final return will be, but the more work the algorithm will have to do.
    """
    def __init__(self,error,xGuess,tolerance,delta):
        self.error     = error
        self.xGuess    = xGuess
        self.tolerance = tolerance
        self.delta     = delta
        
#    def find(self):
#        """Finds an x value that gives a desired y value based on a given error
#        function within a given tolerance. This method uses Newton's method.
#
#        Parameters
#        -----
#        None
#
#        Returns
#        -----
#        The estimated x value that gives the desired y value within a tolerance.
#        """
#        error=self.error
#        xGuess=self.xGuess
#        tolerance=self.tolerance
#        delta=self.delta
#        
#        err = error(xGuess)
#
#        while abs(err)>tolerance:
##            print( 'Abs(error(xGuess)) = ' + repr(abs(error(xGuess))))
##            print( 'xGuess = ' + repr(xGuess))
#            print( 'err = ' + repr(err))
#            errDelta = error(xGuess+delta)
##            print( 'errDlta = ' + repr(errDelta))
#            Eprime = (-err+errDelta)/delta
#            xGuess = xGuess-(err/Eprime)
#            err = error(xGuess)
#            
#        return xGuess
    
    @staticmethod
    def find(error, xGuess, tolerance, delta):
        """Finds an x value that gives a desired y value based on a given error
        function within a given tolerance. This method uses Newton's method.

        Parameters
        -----
        None

        Returns
        -----
        The estimated x value that gives the desired y value within a tolerance.
        """

        
        err = error(xGuess)

        while abs(err)>tolerance:
#            print( 'Abs(error(xGuess)) = ' + repr(abs(error(xGuess))))
#            print( 'xGuess = ' + repr(xGuess))
            print( 'err = ' + repr(err))
            errDelta = error(xGuess+delta)
#            print( 'errDlta = ' + repr(errDelta))
            Eprime = (-err+errDelta)/delta
            xGuess = xGuess-(err/Eprime)
            err = error(xGuess)
            
        return xGuess
        
class GoldenSection(object):
    """
    """
    def __init__(self,error,bounds,tolerance):
        """
        """
        self.error=error
        self.bounds=bounds
        self.tolerance=tolerance
        self.tl=bounds[0]
        self.tr=bounds[1]
        w=abs(self.tr-self.tl)
        l=((3-np.sqrt(5))/2.)*w
        self.tm=self.tl+l
        tbar=w-l*2
        self.t=self.tm+tbar
        
    def find(self):
        """Finds an x value that gives a desired y value based on a given error
        function within a given tolerance. This method uses Newton's method.

        Parameters
        -----
        None

        Returns
        -----
        The estimated x value that gives the desired y value within a tolerance.
        """
        error=self.error
        tolerance=self.tolerance
        tl=self.tl
        tr=self.tr
        tm=self.tm
        t=self.t
        


#        tl = 20717821
#        tm = 20492058
#        tr = 21083113
#        t  = 20578292    
 
        errtl = error(tl)
        errtm = error(tm)
        errtr = error(tr)
        errt = error(t)
#        n=0
       
        while abs(tl-tr)>tolerance:
            
#            n+=1
#            print( n)
            oldBrackets=cp.deepcopy([tl,tm,t,tr])
            
            if tm-tl > tr-tm:
                leftBig=True
                rightBig=False
            else:
                leftBig=False
                rightBig=True
                
            if rightBig:
                if errt>errtm:
#                    print( 'right less')
                    tr=t
                    errtr=errt
                    
                    w=tr-tl
                    l=((3-np.sqrt(5))/2.)*w
                    tbar=w-l*2
                    
                    t=tm-tbar
                elif errt<errtm:
#                    print( 'right great')
                    tl=tm
                    errtl=errtm
                    tm=t
                    errtm=errt
                    
                    w=tr-tl
                    l=((3-np.sqrt(5))/2.)*w
                    tbar=w-l*2
                    
                    t=tm+tbar    
                else: print( 'They be equal on right!')
            elif leftBig:
                if errt>errtm:
#                    print( 'left less')
                    tl=t
                    errtl=errt
                    
                    w=tr-tl
                    l=((3-np.sqrt(5))/2.)*w
                    tbar=w-l*2
                    
                    t=tm+tbar
                elif errt<errtm:
#                    print( 'left great')
                    tr=tm
                    errtr=errtm
                    tm=t
                    errtm=errt
                    
                    w=tr-tl
                    l=((3-np.sqrt(5))/2.)*w
                    tbar=w-l*2
                    
                    t=tm-tbar
                else: print( 'They be equal on left!')
            else: print( 'Error')
#            print( tr-tl)
            if tr-tl<0:
                print( 'Broke!')
            
            errt = error(t)
        return t
