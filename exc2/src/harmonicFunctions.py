"""
Author: Gustav Kjellberg
Email: kjellberg.p.gustav@gmail.com
Licence: MIT
Creation data: 22 Sept 2020
Last modified: 23 Sept 2020
"""

import math as math

import numpy as np
import pandas as pd


class HarmonicFunctions():
    """[summary]
        A class that creates a harmonic function.
    """    
    def __init__(self, t_start, t_stop):
        """[summary]
            Class initializer
        Args:
            t_start ([int]): [The time t where the data starts]
            t_stop ([int]): [The time t where the data ends]
        """        
        self._t_start = t_start
        self._t_stop = t_stop
        self._global_minima_value = math.inf
        self._step_size = 0.01
        self._global_minima_t = None
        self._periodicity = None
        self._index_of_first_minima = None

    def find_period(self, h_0, h_1, value_t, value_t_minus_1,t):
        """[summary]
            Find the period of the function given that we have enough of a span.
        Args:
            h_0 ([float]): [function value at t_start]
            h_1 ([float]): [function value at t_start+1 step size]
            value_t ([float]): [function value at current t]
            value_t_minus_1 ([float]): [function value at previous t]
            t ([float]): [current t]
        """        
        if math.isclose(value_t, h_0, abs_tol=0.00001): #rounding errors in python, accepts a small diff.
            direction_start = False if h_1-h_0 < 0 else True
            direction_end = False if value_t-value_t_minus_1 < 0 else True
            if direction_start is direction_end:
                self._periodicity = t-self._t_start

    def check_global_minima(self,value, t, index):
        """[summary]
            Method that finds the global (gurantueed in this case due to periodicity) minima of the function. Also funds the period.
        Args:
            value ([float]): [Value of h(t) at t]
            t ([gloat]): [Current t]
            index ([int]): [Index of current t and h(t)]
        """        
        if value < self._global_minima_value: # If the value is smaller we have a new minima
            self._global_minima_value = value 
        elif value == self._global_minima_value: # If value is equal we're at a new minima pos
            if self._global_minima_t is not None and self._periodicity is None: # If periodicity is None then we need to set it
                self._periodicity = t-self._global_minima_t
        else:    
            if self._index_of_first_minima is None:
                self._index_of_first_minima = index-1 # minus 1 bcs we have started to increase again.
                self._global_minima_t = t-self._step_size

    def make_harmonic(self):
        """[summary]
            Method that creates the function of the object
        Returns:
            [{step_size:float, period:float, data:{t:[float], h_t:[float]}}]: [A dict containing the function values, time t, period and step size]
        """        
        function_values_and_time = {'t': [], 'h_t':[]}
        values_of_t = np.arange(self._t_start, self._t_stop+self._step_size, self._step_size) #for-loop instead of a lamda expression for easier interpretation
        for index, t in enumerate(values_of_t):
            function_value_at_t = 3*math.pi*math.exp(-(self.lambda_method(t)))
            if index > 1 and self._periodicity is None:
                self.find_period(function_values_and_time['h_t'][0],function_values_and_time['h_t'][1], \
                                function_value_at_t, function_values_and_time['h_t'][index-1],t)
            #self.check_global_minima(function_value_at_t, t, index)
            function_values_and_time['t'].append(t)
            function_values_and_time['h_t'].append(function_value_at_t)
        if self._periodicity is None: # This is necessary bcs we might look at an interval that is less than a period.
            self._periodicity = index
        return {'step_size': self._step_size, 'period': self._periodicity, 'data':function_values_and_time}
    
    def lambda_method(self,t):
        """[summary]
            Method that performs the mathematics of function lamda described in the excercise.
        Args:
            t ([float]): [Time t]

        Returns:
            [float]: [value of lamda at time t]
        """        
        return 5*math.sin(2*math.pi*1*t) # I don't see the value of 1 here but this is how lamda is defined in the exercise.
