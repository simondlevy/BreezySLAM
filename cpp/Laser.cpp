/**
* 
* BreezySLAM: Simple, efficient SLAM in C++
*
* Laser.cpp - C++ code for Laser model class
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

#include "Laser.hpp"
#include "coreslam.h"


Laser::Laser(
    int scanSize,
    float scanRateHz,
    float detection_angle_degrees,
    float distance_no_detection_mm,
    int detection_margin,
    float offset_mm
    )
{    
    this->laser = new laser_t;
    
    this->laser->scan_size = scanSize;
    this->laser->scan_rate_hz = scanRateHz;
    this->laser->detection_angle_degrees = detection_angle_degrees;
    this->laser->distance_no_detection_mm = distance_no_detection_mm;
    this->laser->detection_margin = detection_margin;
    this->laser->offset_mm = offset_mm;
}


Laser::Laser(void)
{
    this->laser = new laser_t;

    this->laser->scan_size = 0;
    this->laser->scan_rate_hz = 0;
    this->laser->detection_angle_degrees = 0;
    this->laser->distance_no_detection_mm = 0;
    this->laser->detection_margin = 0;
    this->laser->offset_mm = 0;   
}


Laser::~Laser(void)
{
    delete this->laser;
}

ostream& operator<< (ostream & out, Laser & laser)
{
    char str[512];
    char lstr[512];
    
    laser_string(*laser.laser, lstr);
    
    sprintf(str, "<%s>", lstr);
    
    out << str;
    
    return out;
}

