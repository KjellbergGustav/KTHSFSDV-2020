"""
Author: Gustav Kjellberg
Email: kjellberg.p.gustav@gmail.com
Licence: MIT
Creation data: 22 Sept 2020
Last modified: 23 Sept 2020
"""

import math
import os
import sys

import matplotlib.pyplot as plt
import numpy as np
from matplotlib import animation


class Graphics():
    """[summary]
        A class for visualizing two-dim data
    """    
    def __init__(self):
        """[summary]
            Class initializer
        """        
        self._values = None

    def add_new_valus(self, values):
        """[summary]
            This methods enables the user to add data to the class.
        Args:
            values ([dict{step_size:float, period:float, data:{t:[float], h_t:[float]}}]): [A dict containing the necessary data to plot the function and limit the shown data to one period]
        """        
        self._values = values

    def plot_2d(self):
        """[summary]
        A 2 dimenstional plot that shows that shows all data at once.
        """        
        function_data = self._values['data']
        t = function_data['t']
        h_t = function_data['h_t']

        plt.plot(t,h_t)
        plt.show()

    def save_fig(self, figure):
        """[summary]
            This method adds the support to save the plot on the users command including naming of the graph.
        Args:
            figure ([pyplot]): [the generated plot]
        """        
        save_fig = input('Do you want to save the figure? [y/n]\n')
        if save_fig == 'y':
            fig_name = input('Enter figure name: ')
            figure.savefig('plots/'+fig_name)
        sys.exit()

    def plot_2d_live(self):
        """[summary]
            A 2 dimenstional plot that shows that update the data \'live\'.
        """        
        function_data = self._values['data'] # function data
        step_size = self._values['step_size'] # step size
        period = self._values['period'] # function periond
        t = function_data['t'] # variable t data
        h_t = function_data['h_t'] # function values at time t

        plt.axes().set(xlim=(t[0], t[-1]), ylim=(h_t[0], max(h_t))) # Limit axes to our data
        plt.ion() #make interactive

        lower_bound_cut_off = 0 # A lower bound so that we only show one period at max
        for index in range(0,len(h_t)):
            if index > period/step_size:
                lower_bound_cut_off+=1
            plt.xlabel('Time (t)')
            plt.ylabel('h(t)')
            plt.plot(t[lower_bound_cut_off:index],h_t[lower_bound_cut_off:index], 'r-', label='h(t) = 3*pi*exp(-5*sin(2*pi*t))\nh({:.2f}) = {:.2f}'.format(t[index], h_t[index]))
            plt.legend(loc='upper left') # place the legend at the top left corner
            plt.show()
            plt.pause(0.01) #Pause for more entertaining animation
            if index == int(math.floor(period/step_size)): # When you have seen one period you should have insight enough to understand the function, being able to stop and save is thus good
                continue_animation = input('You\'ve just seen one period of the function, continue? [y/n]\n')
                if continue_animation != 'y':
                    self.save_fig(plt)
            if index == len(h_t)-1:
                if index < int(math.floor(period/step_size)):
                    print('You\'ve entered values of t spanning less than one period.')    
                else:
                    print('There\'s no more data left')
                self.save_fig(plt)
            plt.clf()
