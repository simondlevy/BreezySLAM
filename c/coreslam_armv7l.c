/*
coreslam_armv7l.c.c ARM Cortex Neon acceleration for CoreSLAM

Copyright (C) 2014 by Simon D. Levy

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


#ifdef _MSC_VER
typedef __int64 int64_t;       /* Define it from MSVC's internal type */
#else
#include <stdint.h>            /* Use the C99 official header */
#endif

#include <math.h>
#include <stdio.h>

#include <arm_neon.h>

#include "coreslam.h"
#include "coreslam_internals.h"

/* Performs one rotation/translation */
static void 
neon_coord_4(
    float32x4_t a_4, 
    float32x4_t b_4,
    float32x4_t x_4, 
    float32x4_t y_4,
    float32x4_t pos_4f, 
    float32x4_t point5_4, 
    int * result)
{
    float32x4_t tmp1 = vmulq_f32(a_4, x_4);
    float32x4_t tmp2 = vmulq_f32(b_4, y_4);
    tmp2 = vaddq_f32(tmp1, tmp2);
    tmp2 = vaddq_f32(tmp2, pos_4f);
    tmp2 = vaddq_f32(tmp2, point5_4);
    int32x4_t c_4 = vcvtq_s32_f32(tmp2);
    vst1q_s32(result, c_4);
}


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


    float32x4_t half_4  = vdupq_n_f32(0.5);

    float32x4_t costheta_4  = vdupq_n_f32(costheta);
    float32x4_t sintheta_4  = vdupq_n_f32(sintheta);
    float32x4_t nsintheta_4 = vdupq_n_f32(-sintheta);

    float32x4_t pos_x_4 = vdupq_n_f32(pos_x_pix);
    float32x4_t pos_y_4 = vdupq_n_f32(pos_y_pix);

    int npoints = 0; /* number of points where scan matches map */
    int64_t sum = 0;
    
    /* Stride by 4 over obstacle points in scan */
    int i = 0;
    for (i=0; i<scan->obst_npoints; i+=4) 
    {        
        /* Duplicate current obstacle point X and Y in 128-bit registers */
        float32x4_t scan_x_4 = vld1q_f32(&scan->obst_x_mm[i]); 
        float32x4_t scan_y_4 = vld1q_f32(&scan->obst_y_mm[i]); 

        /* Compute X coordinate of 4 rotated / translated points at once */
        int xarr[4];
        neon_coord_4(costheta_4, nsintheta_4, scan_x_4, scan_y_4, pos_x_4, half_4, xarr);

        /* Compute Y coordinate of 4 rotated / translated points at once */
        int yarr[4];
        neon_coord_4(sintheta_4, costheta_4,  scan_x_4, scan_y_4, pos_y_4, half_4, yarr);

        /* Handle rotated/translated points serially */
        int j;
        for (j=0; j<4 && (i+j)<scan->obst_npoints; ++j)
        {
            int x = xarr[j];
            int y = yarr[j];

	    /* Add point if in map bounds */
	    if (x >= 0 && x < map->size_pixels && y >= 0 && y < map->size_pixels) 
	    {
		    sum += map->pixels[y * map->size_pixels + x];
		    npoints++;
	    }
	}
    }

    return npoints ? (int)(sum * 1024 / npoints) : -1;  
}
