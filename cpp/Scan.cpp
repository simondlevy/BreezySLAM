/**
* 
* \mainpage BreezySLAM: Simple, efficient SLAM in C++
*
* Scan.cpp - implementation for Scan class
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
#include <string.h>

#include <iostream>
#include <vector>
using namespace std; 

#include "coreslam.h"

#include "PoseChange.hpp"
#include "Laser.hpp"
#include "Scan.hpp"

/**
* A class for Lidar scans.
*/

Scan::Scan(Laser * laser, int span)
{
    this->init(laser, span);
}

Scan::Scan(Laser * laser) 
{
    this->init(laser, 1);
}


void Scan::init(Laser * laser, int span)
{
    this->scan = new scan_t;
    
    scan_init(
            this->scan, 
            span, 
            laser->scan_size,
            laser->scan_rate_hz,                
            laser->detection_angle_degrees,     
            laser->distance_no_detection_mm,    
            laser->detection_margin,               
            laser->offset_mm);
}

Scan::~Scan(void)
{
    scan_free(this->scan);
    delete this->scan;
}


void 
Scan::update(
    int * scanvals_mm, 
    double hole_width_millimeters,
    PoseChange & poseChange)
{
    scan_update(
        this->scan,
        NULL, // no support for angles/interpolation yet
        scanvals_mm,
        this->scan->size,// no support for angles/interpolation yet
        hole_width_millimeters,
        poseChange.dxy_mm,
        poseChange.dtheta_degrees);
}
void 
Scan::update(
    int * scanvals_mm, 
    double hole_width_millimeters)
{
    PoseChange zeroPoseChange;
    this->update(scanvals_mm, hole_width_millimeters, zeroPoseChange);
}

ostream& operator<< (ostream & out, Scan & scan)
{
    char str[512];
    
    scan_string(*scan.scan, str);
    
    out << str;
    
    return out;
}


