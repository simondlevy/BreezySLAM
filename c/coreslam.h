/*

coreslam.h adapted from CoreSLAM.h downloaded from openslam.org on 01 January 2014.  
Contains efficient implementations of scan and map structures.

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

/* Default parameters --------------------------------------------------------*/

static const int    DEFAULT_MAP_QUALITY         = 50; /* out of 255 */
static const double DEFAULT_HOLE_WIDTH_MM       = 600;

static const double DEFAULT_SIGMA_XY_MM         = 100;
static const double DEFAULT_SIGMA_THETA_DEGREES = 20;

static const double DEFAULT_MAX_SEARCH_ITER     = 1000;


/* Core types --------------------------------------------------------------- */

typedef struct position_t
{
    double x_mm;
    double y_mm;
    double theta_degrees;
    
} position_t;

typedef unsigned short pixel_t;

typedef struct map_t {
    
    pixel_t * pixels;
    int size_pixels;
    double size_meters;
    
    double scale_pixels_per_mm;
    
} map_t;


typedef struct scan_t
{
    double * x_mm;
    double * y_mm;
    int * value;
    int npoints;
    int span;

    int size;                           /* number of rays per scan */
    double rate_hz;                     /* scans per second */
    double detection_angle_degrees;     /* e.g. 240, 360 */
    double distance_no_detection_mm;    /* default value when the laser returns 0 */
    int detection_margin;               /* first scan element to consider */
    double offset_mm;                   /* position of the laser wrt center of rotation */

    /* for angle/distance interpolation */
    void * interpolation;
     
    /* for SSE */
    float * obst_x_mm;
    float * obst_y_mm;
    int obst_npoints;
        
} scan_t;

/* Exported functions ------------------------------------------------------- */

#ifdef __cplusplus 
extern "C" 
{
#endif
    
int * 
int_alloc(
    int size);

float * 
float_alloc(
    int size);

void 
map_init(
    map_t * map, 
    int size_pixels, 
    double size_meters);

void
map_free(
    map_t * map);

void map_string(
    map_t map,
    char * str);

void
map_update(
    map_t * map, 
    scan_t * scan, 
    position_t position,
    int map_quality, 
    double hole_width_mm);

void scan_init(
    scan_t * scan, 
    int span,
    int size,
    double scan_rate_hz,                
    double detection_angle_degrees,     
    double distance_no_detection_mm,    
    int detection_margin,               
    double offset_mm);

void 
scan_free(
    scan_t * scan);

void scan_string(
    scan_t scan,
    char * str);

void 
scan_update(
    scan_t * scan, 
    float * lidar_angles_deg,
    int   * lidar_distances_mm, 
    int     scan_size, 
    double hole_width_mm,
    double velocities_dxy_mm,
    double velocities_dtheta_degrees);

void
map_get(
    map_t * map, 
    char * bytes);

void
map_set(
    map_t * map, 
    char * bytes);
    
/* Returns -1 for infinity */
int 
distance_scan_to_map(
    map_t *  map,
    scan_t * scan,
    position_t position);


/* Random-Mutation Hill-Climbing search */
position_t 
rmhc_position_search(
    position_t start_pos,
	map_t * map,
    scan_t * scan,
	double sigma_xy_mm,
	double sigma_theta_degrees,
	int max_search_iter,
	void * randomizer);

#ifdef __cplusplus 
}
#endif

