#!/usr/bin/env python3

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
MAP_SIZE_METERS          = 32

from breezyslam.algorithms import Deterministic_SLAM, RMHC_SLAM
from mines import MinesLaser, Rover, load_data
from roboviz import MapVisualizer

from sys import argv, exit
from time import sleep
from threading import Thread

def threadfunc(robot, slam, timestamps, lidars, odometries, mapbytes, pose):
    '''
    Threaded function runs SLAM, setting the map bytes and robot pose for display
    on the main thread.
    '''

    # Initialize time for delay
    prevtime = 0

    # Loop over scans    
    for scanno in range(len(lidars)):

        if odometries is None:
                  
             # Update SLAM with lidar alone
            slam.update(lidars[scanno])

        else:
        
            # Convert odometry to velocities
            velocities = robot.computePoseChange(odometries[scanno])

            # Update SLAM with lidar and velocities
            slam.update(lidars[scanno], velocities)

        # Get new position
        pose[0],pose[1],pose[2] = slam.getpos()    

        # Get new map
        slam.getmap(mapbytes)

        # Add delay to yield to main thread
        currtime = timestamps[scanno] / 1.e6 # Convert usec to sec
        if prevtime > 0:
            sleep(currtime-prevtime)
        prevtime = currtime
    
def main():
	    
    # Bozo filter for input args
    if len(argv) < 4:
        print('Usage:   %s <dataset> <use_odometry> [random_seed]' % argv[0])
        print('Example: %s exp2 1 9999' % argv[0])
        exit(1)
    
    # Grab input args
    dataset = argv[1]
    use_odometry  =  True if int(argv[2]) else False
    seed =  int(argv[3]) if len(argv) > 3 else 0

    # Allocate byte array to receive map updates
    mapbytes = bytearray(MAP_SIZE_PIXELS * MAP_SIZE_PIXELS)
    
	# Load the data from the file    
    timestamps, lidars, odometries = load_data('.', dataset)
    
    # Build a robot model if we want odometry
    robot = Rover() if use_odometry else None
        
    # Create a CoreSLAM object with laser params and optional robot object
    slam = RMHC_SLAM(MinesLaser(), MAP_SIZE_PIXELS, MAP_SIZE_METERS, random_seed=seed) \
           if seed \
           else Deterministic_SLAM(MinesLaser(), MAP_SIZE_PIXELS, MAP_SIZE_METERS)

    # Set up a SLAM display, named by dataset
    viz = MapVisualizer(MAP_SIZE_PIXELS, MAP_SIZE_METERS, dataset)

    # Pose will be modified in our threaded code
    pose = [0,0,0]

    # Launch the data-collection / update thread
    thread = Thread(target=threadfunc, args=(robot, slam, timestamps, lidars, odometries if use_odometry else None, mapbytes, pose))
    thread.daemon = True
    thread.start()
    
    # Loop forever,displaying current map and pose
    while True:

        # Display map and robot pose, exiting gracefully if user closes it
        if not viz.display(pose[0]/1000., pose[1]/1000., pose[2], mapbytes):
            exit(0)
                    
main()
