/**
* 
* BreezySLAM: Simple, efficient SLAM in C++
*
* Scan.hpp - header for Scan class
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

class Velocities;
class Laser;


/**
* A class for Lidar scans.
*/
class Scan  
{
    friend class Map;
    friend class CoreSLAM;
    friend class RMHC_SLAM;
        
public:
    
/**
* Builds a Scan object.
* @param laser laser parameters
* 
*/
Scan(Laser * laser);

/**
* Builds a Scan object.
* @param laser laser parameters
* @param span supports spanning laser scan to cover the space better.
* 
*/
Scan(Laser * laser, int span);


/**
* Deallocates this Scan object.
* 
*/
~Scan(void);


/**
* Updates this Scan object with new values from a Lidar scan.
* @param scanvals_mm scanned Lidar distance values in millimeters
* @param hole_width_millimeters hole width in millimeters
* 
*/
void 
update(
    int * scanvals_mm, 
    double hole_width_millimeters);

    
/**
* Updates this Scan object with new values from a Lidar scan.
* @param scanvals_mm scanned Lidar distance values in millimeters
* @param hole_width_millimeters hole width in millimeters
* @param velocities forward velocity and angular velocity of robot at scan time
* 
*/
void 
update(
    int * scanvals_mm, 
    double hole_width_millimeters,
    Velocities & velocities);

friend ostream& operator<< (ostream & out, Scan & scan);

private:
    
    struct scan_t * scan;
    
    void init(Laser * laser, int span);
};

