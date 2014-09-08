/**
* 
* \mainpage BreezySLAM: Simple, efficient SLAM in C++
*
* Map.cpp - implementation for Map class
*
* Copyright (C) 2014 Simon D. Levy

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

#include <stdio.h>
#include <math.h>
#include <stdlib.h>

#include <iostream>
using namespace std; 

#include "coreslam.h"

#include "Scan.hpp"
#include "Position.hpp"
#include "Map.hpp"

Map::Map(int size_pixels, double size_meters)
{
    this->map = new map_t;
    map_init(this->map, size_pixels, size_meters);
}

Map::~Map(void)
{
    map_free(this->map);
    delete this->map;
}

void Map::update(
    Scan & scan, 
    Position & position, 
    int quality, 
    double hole_width_mm)
{    
    position_t cpos;
    cpos.x_mm = position.x_mm;
    cpos.y_mm = position.y_mm;
    cpos.theta_degrees = position.theta_degrees;
    
    map_update(this->map, scan.scan, cpos, quality, hole_width_mm);
}

void Map::get(char * bytes)
{
    map_get(this->map, bytes);
}


ostream& operator<< (ostream & out, Map & map)
{
    char str[512];
    
    map_string(*map.map, str);
    
    out << str;
    
    return out;
}


