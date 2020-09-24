"""
Author: Gustav Kjellberg
Email: kjellberg.p.gustav@gmail.com
Licence: MIT
Creation data: 22 Sept 2020
Last modified: 23 Sept 2020
"""

import sys

from graphics import Graphics
from harmonicFunctions import HarmonicFunctions


def main():
    """[summary]
        Main function that creates class objects for creating a predefined function and the displaying it in a graph
    """    
    t_start = input("Please enter t_0:\n")
    t_stop = input("Please enter t_n:\n")
    t_start = int(t_start)
    t_stop = int(t_stop)
    harmonic_functions =HarmonicFunctions(t_start, t_stop)
    graphics = Graphics()
    function_properties_and_data = harmonic_functions.make_harmonic()
    graphics.add_new_valus(function_properties_and_data)
    #graphics.plot_2d()
    graphics.plot_2d_live()



if __name__ == '__main__':
    main()
