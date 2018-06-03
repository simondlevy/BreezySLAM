/**
* 
* PoseChange.hpp - C++ header for PoseChange class
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

#include <iostream>
#include <vector>
using namespace std; 


/**
* A class representing the forward and angular poseChange of a robot.
*/
class PoseChange 
{    
    friend class Scan;
    
public:
    
/**
* Creates a new PoseChange object with specified poseChange.
*/
PoseChange(double dxy_mm, double dtheta_degrees, double dtSeconds)
{
    this->dxy_mm = dxy_mm;
    this->dtheta_degrees = dtheta_degrees;
    this->dt_seconds = dtSeconds;
}

/**
* Creates a new PoseChange object with zero poseChange.
*/
PoseChange(void)
{
    this->dxy_mm = 0;
    this->dtheta_degrees = 0;
    this->dt_seconds = 0;
}



/**
* Updates this PoseChange object.
* @param dxy_mm new forward distance traveled in millimeters
* @param dtheta_degrees new angular rotation in degrees
* @param dtSeconds time in seconds since last poseChange
*/
void update(double dxy_mm, double dtheta_degrees, double dtSeconds)
{
    double velocity_factor = (dtSeconds > 0) ?  (1 / dtSeconds) : 0;
    
    this->dxy_mm = dxy_mm * velocity_factor;
    
    this->dtheta_degrees = dtheta_degrees * velocity_factor;
}

friend ostream& operator<< (ostream & out, PoseChange & poseChange)
{
    char str[100];
    
    sprintf(str, "<dxy=%7.0f mm dtheta = %+3.3f degrees dt = %f s",
        poseChange.dxy_mm, 
        poseChange.dtheta_degrees,
        poseChange.dt_seconds);
    
    out << str;
    
    return out;
}


double dxy_mm;
double dtheta_degrees;
double dt_seconds;
};
