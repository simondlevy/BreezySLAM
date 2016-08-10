#!/usr/bin/env python

'''
logdemo.py : BreezySLAM Python demo.  Reads logfile with odometry and scan data
             from Paris Mines Tech and displays showing robot pose and map in 
             real time.
             
For details see

    @inproceedings{coreslam-2010,
      author    = {Bruno Steux and Oussama El Hamzaoui},
      title     = {CoreSLAM: a SLAM Algorithm in less than 200 lines of C code},
      booktitle = {11th International Conference on Control, Automation, 
                   Robotics and Vision, ICARCV 2010, Singapore, 7-10 
                   December 2010, Proceedings},
      pages     = {1975-1979},
      publisher = {IEEE},
      year      = {2010}
    }
                 
Copyright (C) 2016 Simon D. Levy and Matt Lubas

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

# Map size, scale
MAP_SIZE_PIXELS          = 800
MAP_SIZE_METERS          =  32

from breezyslam.algorithms import Deterministic_SLAM, RMHC_SLAM
from mines import MinesLaser, Rover, load_data
from pltslamshow import SlamShow

from sys import argv, exit

def main():
	    
    # Bozo filter for input args
    if len(argv) < 3:
        print('Usage:   %s <dataset> <use_odometry> <random_seed>' % argv[0])
        print('Example: %s exp2 1 9999' % argv[0])
        exit(1)
    
    # Grab input args
    dataset = argv[1]
    use_odometry  =  True if int(argv[2]) else False
    seed =  int(argv[3]) if len(argv) > 3 else 0
    
	# Load the data from the file    
    lidars, odometries = load_data('.', dataset)
    
    # Build a robot model if we want odometry
    robot = Rover() if use_odometry else None
        
    # Create a CoreSLAM object with laser params and optional robot object
    slam = RMHC_SLAM(MinesLaser(), MAP_SIZE_PIXELS, MAP_SIZE_METERS, random_seed=seed) \
           if seed \
           else Deterministic_SLAM(MinesLaser(), MAP_SIZE_PIXELS, MAP_SIZE_METERS)
           
    # Report what we're doing
    nscans = len(lidars)
    
    # Create a byte array to receive the computed maps
    mapbytes = bytearray(MAP_SIZE_PIXELS * MAP_SIZE_PIXELS)

    # Set up a SLAM display
    display = SlamShow(MAP_SIZE_PIXELS, MAP_SIZE_METERS*1000/MAP_SIZE_PIXELS, 'SLAM')
    
    # Loop over scans    
    for scanno in range(nscans):
    
        if use_odometry:
                  
            # Convert odometry to velocities
            velocities = robot.computeVelocities(odometries[scanno])
                                 
            # Update SLAM with lidar and velocities
            slam.update(lidars[scanno], velocities)
            
        else:
        
            # Update SLAM with lidar alone
            slam.update(lidars[scanno])
                    
        # Get new position
        x_mm, y_mm, theta_degrees = slam.getpos()    

        print(scanno, x_mm, y_mm, theta_degrees)

        # Get current map    
        slam.getmap(mapbytes)

        # Display map and robot pose
        display.displayMap(mapbytes)
        display.displayRobot((x_mm, y_mm, theta_degrees))

        # Exit gracefully if user closes display
        key = display.refresh()
        if key != None and (key&0x1A):
            exit(0)
 
        # XXX Add delay for real-time plot
    
            
# Helpers ---------------------------------------------------------        

def mm2pix(mm):
        
    return int(mm / (MAP_SIZE_METERS * 1000. / MAP_SIZE_PIXELS))  
    
                    
main()
