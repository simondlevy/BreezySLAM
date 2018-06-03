#!/usr/bin/env python3

'''
log2pkl.py : BreezySLAM Python demo.  Reads logfile with odometry and scan data
             from Paris Mines Tech and pickles the map file for loading by another
             program
             
Copyright (C) 2014 Simon D. Levy

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
from progressbar import ProgressBar

from sys import argv, exit, stdout
from time import time
import pickle

def main():
	    
    # Bozo filter for input args
    if len(argv) < 3:
        print('Usage:   %s <dataset> <use_odometry> [random_seed]' % argv[0])
        print('Example: %s exp2 1 9999' % argv[0])
        exit(1)
    
    # Grab input args
    dataset = argv[1]
    use_odometry  =  True if int(argv[2]) else False
    seed =  int(argv[3]) if len(argv) > 3 else 0
    
	# Load the data from the file, ignoring timestamps
    _, lidars, odometries = load_data('.', dataset)
    
    # Build a robot model if we want odometry
    robot = Rover() if use_odometry else None
        
    # Create a CoreSLAM object with laser params and optional robot object
    slam = RMHC_SLAM(MinesLaser(), MAP_SIZE_PIXELS, MAP_SIZE_METERS, random_seed=seed) \
           if seed \
           else Deterministic_SLAM(MinesLaser(), MAP_SIZE_PIXELS, MAP_SIZE_METERS)
           
    # Report what we're doing
    nscans = len(lidars)
    print('Processing %d scans with%s odometry / with%s particle filter...' % \
        (nscans, \
         '' if use_odometry else 'out', '' if seed else 'out'))
    progbar = ProgressBar(0, nscans, 80)
    
    # Start with an empty trajectory of positions
    trajectory = []

    # Start timing
    start_sec = time()
    
    # Loop over scans    
    for scanno in range(nscans):
    
        if use_odometry:
                  
            # Convert odometry to velocities
            velocities = robot.computePoseChange(odometries[scanno])
                                 
            # Update SLAM with lidar and velocities
            slam.update(lidars[scanno], velocities)
            
        else:
        
            # Update SLAM with lidar alone
            slam.update(lidars[scanno])
                    
        # Get new position
        x_mm, y_mm, theta_degrees = slam.getpos()    
        
        # Add new position to trajectory
        trajectory.append((x_mm, y_mm))
        
        # Tame impatience
        progbar.updateAmount(scanno)
        stdout.write('\r%s' % str(progbar))
        stdout.flush()

    # Report elapsed time
    elapsed_sec = time() - start_sec
    print('\n%d scans in %f sec = %f scans / sec' % (nscans, elapsed_sec, nscans/elapsed_sec))
                    
                                
    # Create a byte array to receive the computed maps
    mapbytes = bytearray(MAP_SIZE_PIXELS * MAP_SIZE_PIXELS)
    
    # Get final map    
    slam.getmap(mapbytes)

    # Pickle the map to a file
    pklname = dataset + '.map'
    print('Writing map to file ' + pklname)
    pickle.dump(mapbytes, open(pklname, 'wb'))
    
            
# Helpers ---------------------------------------------------------        

def mm2pix(mm):
        
    return int(mm / (MAP_SIZE_METERS * 1000. / MAP_SIZE_PIXELS))  
    
                    
main()
