/*
algorithms.cpp - C++ CoreSLAM algorithms

Copyright (C) 2013 Simon D. Levy

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
*/

#include "coreslam.h"
#include "random.h"

#include "Position.hpp"
#include "Map.hpp"
#include "Laser.hpp"
#include "Scan.hpp"
#include "Velocities.hpp"
#include "WheeledRobot.hpp"

#include "algorithms.hpp"

// Local helpers -------------------------------------------------------------------------------------------------------

static void Position2position_t(Position & cpp_pos, struct position_t * c_pos)
{
    c_pos->x_mm = cpp_pos.x_mm;
    c_pos->y_mm = cpp_pos.y_mm;
    c_pos->theta_degrees = cpp_pos.theta_degrees;
}

// CoreSLAM class -------------------------------------------------------------------------------------------------------

int CoreSLAM::distanceScanToMap(
    Scan & scan, 
    Map & map,
    Position & position)
{
   position_t pos_c;
   Position2position_t(position, &pos_c);
   
   return distance_scan_to_map(map.map, scan.scan, pos_c);
}


CoreSLAM::CoreSLAM(
    Laser & laser, 
    int map_size_pixels, 
    double map_size_meters)
{
    // Set default params
    this->map_quality = DEFAULT_MAP_QUALITY;
    this->hole_width_mm = DEFAULT_HOLE_WIDTH_MM;   
    
    // Store laser for later
    this->laser = new Laser(laser);
    
    // Initialize velocities (dxyMillimeters, dthetaDegrees, dtSeconds) for odometry
    this->velocities = new Velocities();

    // Initialize a scan for computing distance to map, and one for updating map
    this->scan_for_mapbuild = this->scan_create(3);
    this->scan_for_distance = this->scan_create(1);
    
    // Initialize the map 
    this->map = new Map(map_size_pixels, map_size_meters);
}

CoreSLAM::~CoreSLAM(void)
{        
    delete this->map;
    delete this->scan_for_distance;
    delete this->scan_for_mapbuild;
    delete this->velocities;
}


void CoreSLAM::update(int * scan_mm, Velocities & velocities)
{             
    // Build a scan for computing distance to map, and one for updating map
    this->scan_update(this->scan_for_mapbuild, scan_mm);
    this->scan_update(this->scan_for_distance, scan_mm);
    
    // Update velocities
    this->velocities->update(velocities.dxy_mm, 
                             velocities.dtheta_degrees,  
                             velocities.dt_seconds);
                             
    // Implementing class updates map and pointcloud
    this->updateMapAndPointcloud(velocities);
}   

void CoreSLAM::update(int * scan_mm) 
{
    Velocities zero_velocities;   
    
    this->update(scan_mm, zero_velocities);
}


void CoreSLAM::getmap(unsigned char * mapbytes)
{
    this->map->get((char *)mapbytes);
}

Scan * CoreSLAM::scan_create(int span)
{
    return new Scan(this->laser, span);
}


void 
CoreSLAM::scan_update(Scan * scan, int * scan_mm)
{
    scan->update(scan_mm, this->hole_width_mm, *this->velocities);
}

SinglePositionSLAM::SinglePositionSLAM(Laser & laser, int map_size_pixels, double map_size_meters) :
CoreSLAM(laser, map_size_pixels, map_size_meters)
{
    this->position = Position(this->init_coord_mm(), this->init_coord_mm(), 0);
}

    
// SinglePositionSLAM class -------------------------------------------------------------------------------------------

void SinglePositionSLAM::updateMapAndPointcloud(Velocities & velocities)
{    
    // Start at current position 
    Position start_pos = this->position;

    // Add effect of velocities
    start_pos.x_mm      += velocities.dxy_mm * this->costheta(),
    start_pos.y_mm      += velocities.dxy_mm *  this->sintheta(),
    start_pos.theta_degrees += velocities.dtheta_degrees;
    
    // Add offset from laser
    start_pos.x_mm += this->laser->offset_mm * this->costheta();
    start_pos.y_mm += this->laser->offset_mm * this->sintheta();
    
    // Get new position from implementing class
    Position new_position = this->getNewPosition(start_pos);
         
    // Update the map with this new position
    this->map->update(*this->scan_for_mapbuild, new_position, this->map_quality, this->hole_width_mm);
   
    // Update the current position with this new position, adjusted by laser offset
    this->position = Position(new_position);
    this->position.x_mm -= this->laser->offset_mm * this->costheta();
    this->position.y_mm -= this->laser->offset_mm * this->sintheta(); 
}
    
Position & SinglePositionSLAM::getpos(void)
{
    return this->position;
}

double SinglePositionSLAM::init_coord_mm(void)
{
    // Center of map
    return 500 * this->map->map->size_meters;
}


double SinglePositionSLAM::theta_radians(void)
{
    return M_PI * this->position.theta_degrees / 180;
}

double SinglePositionSLAM::costheta(void)
{
    return cos(this->theta_radians());
}

double SinglePositionSLAM::sintheta(void)
{
    return sin(this->theta_radians());
}


// RMHC_SLAM class ------------------------------------------------------------------------------------------------------

RMHC_SLAM::RMHC_SLAM(Laser & laser, int map_size_pixels, double map_size_meters, unsigned random_seed) :
SinglePositionSLAM(laser, map_size_pixels, map_size_meters)
{    
    this->sigma_xy_mm = DEFAULT_SIGMA_XY_MM;
    this->sigma_theta_degrees = DEFAULT_SIGMA_THETA_DEGREES;
    
    this->max_search_iter = DEFAULT_MAX_SEARCH_ITER;
    
    this->randomizer = random_new(random_seed);
}

RMHC_SLAM::~RMHC_SLAM(void)
{
    free(this->randomizer);
}

Position RMHC_SLAM::getNewPosition(Position & start_pos)
{
    // Search for a new position if indicated
    Position likeliest_position = start_pos;
    if (this->randomizer)
    {
        // Use C to find likeliest position
        position_t start_pos_c;
        Position2position_t(start_pos, &start_pos_c);
        position_t c_likeliest_position = 
        rmhc_position_search(
            start_pos_c,
            this->map->map,
            this->scan_for_distance->scan,
            this->sigma_xy_mm,
            this->sigma_theta_degrees,
            this->max_search_iter,
            this->randomizer);    
        
        // Convert back to C++ object
        likeliest_position = 
        Position(
            c_likeliest_position.x_mm, 
            c_likeliest_position.y_mm, 
            c_likeliest_position.theta_degrees); 
    }

    return likeliest_position;
}

// DeterministicSLAM class ---------------------------------------------------------------------------------------------

Deterministic_SLAM::Deterministic_SLAM(Laser & laser, int map_size_pixels, double map_size_meters) :
SinglePositionSLAM(laser, map_size_pixels, map_size_meters)
{
}

Position Deterministic_SLAM::getNewPosition(Position & start_pos)
{
    return Position(start_pos.x_mm, start_pos.y_mm, start_pos.theta_degrees);
}


        
