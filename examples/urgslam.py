#!/usr/bin/env python3

'''
urgslam.py : BreezySLAM Python with Hokuyo URG04LX Lidar
                 
Copyright (C) 2015 Simon D. Levy

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

MAP_SIZE_PIXELS         = 500
MAP_SIZE_METERS         = 10
LIDAR_DEVICE            = '/dev/ttyACM0'

from breezyslam.algorithms import RMHC_SLAM
from breezyslam.sensors import URG04LX as LaserModel

from breezylidar import URG04LX as Lidar

from roboviz import MapVisualizer
       
if __name__ == '__main__':

    # Connect to Lidar unit
    lidar = Lidar(LIDAR_DEVICE)

    # Create an RMHC SLAM object with a laser model and optional robot model
    slam = RMHC_SLAM(LaserModel(), MAP_SIZE_PIXELS, MAP_SIZE_METERS)

    # Set up a SLAM display
    viz = MapVisualizer(MAP_SIZE_PIXELS, MAP_SIZE_METERS, 'SLAM')

    # Initialize empty map
    mapbytes = bytearray(MAP_SIZE_PIXELS * MAP_SIZE_PIXELS)

    while True:

        # Update SLAM with current Lidar scan
        slam.update(lidar.getScan())

        # Get current robot position
        x, y, theta = slam.getpos()

        # Get current map bytes as grayscale
        slam.getmap(mapbytes)

        # Display map and robot pose, exiting gracefully if user closes it
        if not viz.display(x/1000., y/1000., theta, mapbytes):
            exit(0)
