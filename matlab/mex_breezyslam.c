/*
 * mex_breezyslam.c : C extensions for BreezySLAM in Matlab
 *
 * Copyright (C) 2014 Simon D. Levy
 *
 * This code is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This code is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this code.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "mex.h"

#include "../c/coreslam.h"
#include "../c/random.h"

#define MAXSTR 100

/* Helpers ------------------------------------------------------------- */

static int _streq(char * s, const char * t)
{
    return !strcmp(s, t);
}

static void _insert_obj_lhs(mxArray *plhs[], void * obj, int pos)
{    
    long * outptr = NULL;
    
    plhs[pos] = mxCreateNumericMatrix(1, 1, mxINT64_CLASS, mxREAL);

    outptr = (long *) mxGetPr(plhs[pos]);
    
    mexMakeMemoryPersistent(obj);
    
    *outptr = (long)obj;
}

static double _get_field(const mxArray * pm, const char * fieldname)
{
    mxArray  * field_array_ptr = mxGetField(pm, 0, fieldname);
    
    return mxGetScalar(field_array_ptr);
}

static long _rhs2ptr(const mxArray * prhs[], int index)
{
    long * inptr = (long *) mxGetPr(prhs[index]);
    
    return *inptr;
}

static scan_t * _rhs2scan(const mxArray * prhs[], int index)
{
    long inptr = _rhs2ptr(prhs, index);
    
    return (scan_t *)inptr;
}

static map_t * _rhs2map(const mxArray * prhs[], int index)
{
    long inptr = _rhs2ptr(prhs, index);
    
    return (map_t *)inptr;
}

static position_t _rhs2pos(const mxArray * prhs[], int index)
{
    position_t position;
    
    position.x_mm          = _get_field(prhs[index], "x_mm");
    position.y_mm          = _get_field(prhs[index], "y_mm");
    position.theta_degrees = _get_field(prhs[index], "theta_degrees");
    
    return position;
}

/* Class methods ------------------------------------------------------- */

static void _map_init(mxArray *plhs[], const mxArray * prhs[])
{
    
    int size_pixels = (int)mxGetScalar(prhs[1]);
    
    double size_meters = mxGetScalar(prhs[2]);
    
    map_t * map = (map_t *)mxMalloc(sizeof(map_t));
    
    map_init(map, size_pixels, size_meters);
    
    _insert_obj_lhs(plhs, map, 0);
}

static void _map_disp(const mxArray * prhs[])
{
    char str[MAXSTR];
    
    map_t * map = _rhs2map(prhs, 1);
    
    map_string(*map, str);
    
    printf("%s\n", str);
}

static void _map_update(const mxArray * prhs[])
{
    map_t  * map  = _rhs2map(prhs, 1);
    
    scan_t * scan = _rhs2scan(prhs, 2);
    
    position_t position = _rhs2pos(prhs, 3);
    
    int map_quality = (int)mxGetScalar(prhs[4]);
    
    double hole_width_mm = mxGetScalar(prhs[5]);
    
    map_update(map, scan, position, map_quality, hole_width_mm);
}

static void _map_get(mxArray *plhs[], const mxArray * prhs[])
{
    map_t * map  = _rhs2map(prhs, 1);
   
    unsigned char * pointer = NULL;
    
    plhs[0] = mxCreateNumericMatrix(map->size_pixels, map->size_pixels,
            mxUINT8_CLASS, mxREAL);
    
    pointer = (unsigned char *)mxGetPr(plhs[0]);
        
    map_get(map, pointer);
}


static void _scan_init(mxArray *plhs[], const mxArray * prhs[])
{
    
    scan_t * scan = (scan_t *)mxMalloc(sizeof(scan_t));
    
    int span = (int)mxGetScalar(prhs[2]);

    const mxArray * laser = prhs[1];
    int scan_size =                  (int)_get_field(laser, "scan_size");
    double scan_rate_hz =             _get_field(laser, "scan_rate_hz");
    double detection_angle_degrees =  _get_field(laser, "detection_angle_degrees");
    double distance_no_detection_mm = _get_field(laser, "distance_no_detection_mm");
    int detection_margin =           (int)_get_field(laser, "detection_margin");
    double offset_mm =                _get_field(laser, "offset_mm");
 
    scan_init(
            scan, 
            span,
            scan_size, 
            scan_rate_hz,                
            detection_angle_degrees,     
            distance_no_detection_mm,    
            detection_margin,               
            offset_mm);
     
    _insert_obj_lhs(plhs, scan, 0);
}

static void _scan_disp(const mxArray * prhs[])
{
    char str[MAXSTR];
    
    scan_t * scan = _rhs2scan(prhs, 1);
    
    scan_string(*scan, str);
    
    printf("%s\n", str);
}

static void _scan_update(const mxArray * prhs[])
{
    scan_t * scan = _rhs2scan(prhs, 1);
    
    int scansize = (int)mxGetNumberOfElements(prhs[2]);
    
    int * lidar_mm = (int *)mxGetPr(prhs[2]);
    
    double hole_width_mm = mxGetScalar(prhs[3]);
    
    double * velocities = mxGetPr(prhs[4]);
    
    /* no support for angles/interpolation yet */
    scan_update(scan, NULL, lidar_mm, scan->size, hole_width_mm, velocities[0], velocities[1]);
}

static void _randomizer_init(mxArray *plhs[], const mxArray * prhs[])
{
    int seed = (int)mxGetScalar(prhs[1]);
    
    void * r = mxMalloc(random_size());
    
    random_init(r, seed);
    
    _insert_obj_lhs(plhs, r, 0);
}

static void _rmhcPositionSearch(mxArray *plhs[], const mxArray * prhs[])
{
    position_t start_pos = _rhs2pos(prhs, 1);
    
    map_t * map = _rhs2map(prhs, 2);
    
    scan_t * scan = _rhs2scan(prhs, 3);
    
    position_t new_pos;
    
    double sigma_xy_mm = mxGetScalar(prhs[5]);
    
    double sigma_theta_degrees = mxGetScalar(prhs[6]);
    
    int max_search_iter = (int)mxGetScalar(prhs[7]);
    
    void * randomizer = (void *)(long)mxGetScalar(prhs[8]);
    
    new_pos =  rmhc_position_search(
            start_pos,
            map,
            scan,
            sigma_xy_mm,
            sigma_theta_degrees,
            max_search_iter,
            randomizer);
    
    plhs[0] = mxCreateDoubleScalar(new_pos.x_mm);
    plhs[1] = mxCreateDoubleScalar(new_pos.y_mm);
    plhs[2] = mxCreateDoubleScalar(new_pos.theta_degrees);
}

/* The gateway function ------------------------------------------------ */
void mexFunction( int nlhs, mxArray *plhs[],
        int nrhs, const mxArray * prhs[])
{
   
    char methodname[MAXSTR];
    
    mxGetString(prhs[0], methodname, 100);

    if (_streq(methodname, "Map_init"))
    {
        _map_init(plhs, prhs);
    }
    
    else if (_streq(methodname, "Map_disp"))
    {
        _map_disp(prhs);
    }
    
    else if (_streq(methodname, "Map_update"))
    {
        _map_update(prhs);
    }
    
    else if (_streq(methodname, "Map_get"))
    {
        _map_get(plhs, prhs);
    }
    
    else if (_streq(methodname, "Scan_init"))
    {        
        _scan_init(plhs, prhs);
    }
    
    else if (_streq(methodname, "Scan_disp"))
    {
        _scan_disp(prhs);
    }
    
    else if (_streq(methodname, "Scan_update"))
    {
        _scan_update(prhs);
    }
    
    else if (_streq(methodname, "Randomizer_init"))
    {
        _randomizer_init(plhs, prhs);
    }
    
    else if (_streq(methodname, "rmhcPositionSearch"))
    {
        _rmhcPositionSearch(plhs, prhs);
    }
}

