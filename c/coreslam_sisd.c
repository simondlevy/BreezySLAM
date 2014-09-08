/*
coreslam_sisd.c SISD default for CoreSLAM when SSE not available. 

Adapted from code in CoreSLAM.c downloaded from openslam.org on 01 January 2014.  

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
along with this code.  If not, see <http:#www.gnu.org/licenses/>.

*/


#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "coreslam.h"
#include "coreslam_internals.h"

int 
distance_scan_to_map(
    map_t *  map,
    scan_t * scan,
    position_t position)
{    
    /* Pre-compute sine and cosine of angle for rotation */
    double position_theta_radians = radians(position.theta_degrees);
    double costheta = cos(position_theta_radians) * map->scale_pixels_per_mm;
    double sintheta = sin(position_theta_radians) * map->scale_pixels_per_mm;
    
    /* Pre-compute pixel offset for translation */
    double pos_x_pix = position.x_mm * map->scale_pixels_per_mm;
    double pos_y_pix = position.y_mm * map->scale_pixels_per_mm;

    int64_t sum = 0; /* sum of map values at those points */
    int npoints = 0; /* number of points where scan matches map */
    
    int i = 0;
    for (i=0; i<scan->npoints; i++) 
    {        
        /* Consider only scan points representing obstacles */
        if (scan->value[i] == OBSTACLE)
        {
            /* Translate and rotate scan point to robot position */
            int x = floor(pos_x_pix + costheta * scan->x_mm[i] - sintheta * scan->y_mm[i] + 0.5);
            int y = floor(pos_y_pix + sintheta * scan->x_mm[i] + costheta * scan->y_mm[i] + 0.5);
         
            /* Add point if in map bounds */
            if (x >= 0 && x < map->size_pixels && y >= 0 && y < map->size_pixels) 
            {
                sum += map->pixels[y * map->size_pixels + x];
                npoints++;
            } 
        }
    } 

    /* Return sum scaled by number of points, or -1 if none */
    return npoints ? (int)(sum * 1024 / npoints) : -1;  
}
