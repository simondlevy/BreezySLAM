#!/usr/bin/env python

'''
benchmark.py : Benchmark breezyslam.algorithms.distanceScanToMap()  
               with lots of random points
                
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

# Size of square mapping area
MAP_SIZE_METERS = 40

# Width of holes (obstacles, walls) in saved map
HOLE_WIDTH_MM   = 600
        
from breezyslam.components import Laser, Map, Scan, Position
from breezyslam.algorithms import distanceScanToMap

from progressbar import ProgressBar

from mines import URG04, load_data
from pgm_utils import pgm_load

from sys import argv, exit, stdout
from time import time
from random import Random

def main():
	    
    # Bozo filter for input args
    if len(argv) < 3:
        print('Usage:   %s <dataset> <npoints>' % argv[0])
        print('Example: %s <exp2> 100000' % argv[0])
        exit(1)
    
    # Grab input args
    dataset = argv[1]
    npoints = int(argv[2])
    
	# Load the Lidar scan data from the log file    
    lidarscans, _ = load_data('.', dataset)
    
    # Load the map from the PGM file
    mapbytes, mapsize_pixels = pgm_load('%s.pgm' % dataset)
    
    # Map is square
    mapsize_pixels = mapsize_pixels[0]
    
    # Create a Map object from the bytes; call it mymap to avoid name collision with Python's built-in map function
    mymap = Map(mapsize_pixels, MAP_SIZE_METERS, mapbytes)
    
    # Create an empty Scan object with URG04_360 laser parameters
    scan = Scan(URG04())
                    
    # Create a progresss bar to report what we're doing
    nscans = len(lidarscans)
    print('Processing %d points for %d scans...' %  (npoints, nscans))
    progbar = ProgressBar(0, nscans, 80)
    
    # Start timing
    start_sec = time()
 
    # Precompute map size in mm
    mapsize_mm = mapsize_pixels * MAP_SIZE_METERS
    
    # Create a random-number generator for points
    rand = Random()
    
    # Loop over scans    
    for scanno in range(nscans):
         
        # Update the Scan object with the Lidar scan values
        scan.update(lidarscans[scanno], HOLE_WIDTH_MM)
        
        # Test a lot of points on this new scan and the pre-loaded map
        for pointno in range(npoints):
            point = Position(rand.uniform(0,mapsize_mm), rand.uniform(0,mapsize_mm), rand.uniform(-180,180))
            distanceScanToMap(mymap, scan, point)
                
        # Tame impatience
        progbar.updateAmount(scanno)
        stdout.write('\r%s' % str(progbar))
        stdout.flush()
        
    # Report results
    elapsed_sec = time() - start_sec
    totalpoints = nscans * npoints
    print('\nProcessed %d points in %f seconds = %d points / sec' % \
           (totalpoints, elapsed_sec, int(totalpoints / elapsed_sec)))
                     
main()
