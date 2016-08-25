'''
pltslamshow.py - Pyplot classes for displaying maps and robots in SLAM projects

Copyright (C) 2016 Simon D. Levy, Matt Lubas, and Alfredo Rwagaju

This code is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as 
published by the Free Software Foundation, either version 3 of the 
License, or (at your option) any later version.

This code is distributed in the hope that it will be useful,     
but WITHOUT ANY WARRANTY without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU Lesser General Public License 
along with this code.  If not, see <http://www.gnu.org/licenses/>.
'''

# Robot display params
ROBOT_HEIGHT_MM = 500
ROBOT_WIDTH_MM  = 300

import matplotlib.pyplot as plt
import matplotlib.cm as colormap
from math import sin, cos, radians
import numpy as np

class SlamShow(object):

    def __init__(self, map_size_pixels, map_scale_mm_per_pixel, title):
    
        # Store constants for update
        self.map_size_pixels = map_size_pixels
        self.map_scale_mm_per_pixel = map_scale_mm_per_pixel

        # Create a byte array to display the map with a color overlay
        self.bgrbytes = bytearray(map_size_pixels * map_size_pixels * 3)
        
        # Make a nice big (10"x10") figure
        fig = plt.figure(figsize=(10,10))

        # Store Python ID of figure to detect window close
        self.figid = id(fig)

        fig.canvas.set_window_title('SLAM')
        plt.title(title)

        self.ax = fig.gca()
        self.ax.set_aspect("auto")
        self.ax.set_autoscale_on(True)

        # Use an "artist" to speed up map drawing
        self.img_artist = None

        # We base the axis on pixels, to support displaying the map
        self.ax.set_xlim([0, map_size_pixels])
        self.ax.set_ylim([0, map_size_pixels])

        # Hence we must relabel the axis ticks to show millimeters
        ticks = np.arange(0,self.map_size_pixels+100,100)
        labels = [str(self.map_scale_mm_per_pixel * tick) for tick in ticks]
        self.ax.xaxis.set_ticks(ticks)
        self.ax.set_xticklabels(labels)
        self.ax.yaxis.set_ticks(ticks)
        self.ax.set_yticklabels(labels)

        self.ax.set_xlabel('X (mm)')
        self.ax.set_ylabel('Y (mm)')

        self.ax.grid(False)

        # Start vehicle at center
        map_center_mm = map_scale_mm_per_pixel * map_size_pixels
        self._add_vehicle(map_center_mm,map_center_mm,0)

    def displayMap(self, mapbytes):

        mapimg = np.reshape(np.frombuffer(mapbytes, dtype=np.uint8), (self.map_size_pixels, self.map_size_pixels))

        if self.img_artist is None:

            self.img_artist = self.ax.imshow(mapimg, cmap=colormap.gray)

        else:

            self.img_artist.set_data(mapimg)

    def setPose(self, x_mm, y_mm, theta_deg):
        '''
        Sets vehicle pose:
        X:      left/right   (cm)
        Y:      forward/back (cm)
        theta:  rotation (degrees)
        '''
        #remove old arrow
        self.vehicle.remove()
        
        #create a new arrow
        self._add_vehicle(x_mm, y_mm, theta_deg)

    def _add_vehicle(self, x_mm, y_mm, theta_deg):

        #Use a very short arrow shaft to orient the head of the arrow
        dx, dy = plt_rotate(0, 0, 0.1, theta_deg)

        s = self.map_scale_mm_per_pixel

        self.vehicle=self.ax.arrow(x_mm/s, y_mm/s, 
                dx, dy, head_width=ROBOT_WIDTH_MM/s, head_length=ROBOT_HEIGHT_MM/s, fc='r', ec='r')

    def refresh(self):                   

        # If we have a new figure, something went wrong (closing figure failed)
        if self.figid != id(plt.gcf()):
            return False

        # Redraw current objects without blocking
        plt.draw()

        # Refresh display, setting flag on window close or keyboard interrupt
        try:
            plt.pause(.01) # Arbitrary pause to force redraw
            return True
        except:
            return False

        return True
    
    # Converts millimeters to pixels
    def mm2pix(self, mm):
        return int(mm / float(self.map_scale_mm_per_pixel))
                
# Helpers -------------------------------------------------------------        

def plt_rotate(x, y, r, deg):
    rad = radians(deg)
    c = cos(rad)
    s = sin(rad)
    dx = r * c
    dy = r * s
    return x+dx, y+dy 
