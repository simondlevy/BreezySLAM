/**
* 
* Map.hpp - header for Map class
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

typedef unsigned short pixel_t;


class Scan;
class Position;

/**
* A class for maps used in SLAM.
*/
class Map  
{
    friend class CoreSLAM;
    friend class SinglePositionSLAM;
    friend class RMHC_SLAM;
        
public:
    
/**
* Builds a square Map object.
* @param size_pixels  size in pixels
* @param size_meters  size in meters
* 
*/
Map(int size_pixels, double size_meters);


/**
* Deallocates this Map object.
* 
*/
~Map(void);


/**
* Puts current map values into bytearray, which should of which should be of 
* this->size map_size_pixels ^ 2.
*/
void get(char * bytes);

/**
* Updates this map object based on new data.
* @param scan a new scan
* @param position a new postion
* @param quality speed with which scan is integerate into map (0 through 255)
* @param hole_width_mm hole width in millimeters
* 
*/
void update(
    Scan & scan, 
    Position & position, 
    int quality, 
    double hole_width_mm);


friend ostream& operator<< (ostream & out, Map & map);

private:
    
    struct map_t * map;
};

