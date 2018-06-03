/**
* 
* \mainpage BreezySLAM: Simple, efficient SLAM in C++
*
* WheeledRobot.cpp - C++ code for WheeledRobot class
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

#include "WheeledRobot.hpp"
#include "PoseChange.hpp"

PoseChange 
WheeledRobot::computePoseChange(
    double timestamp, 
    double left_wheel_odometry, 
    double right_wheel_odometry)
{                
    PoseChange poseChange;
    
    double timestamp_seconds_curr, left_wheel_degrees_curr, right_wheel_degrees_curr;
    
    this->extractOdometry(timestamp, left_wheel_odometry, right_wheel_odometry, 
        timestamp_seconds_curr, left_wheel_degrees_curr, right_wheel_degrees_curr);
    
    if (this->timestamp_seconds_prev > 0)
    {             
        double left_diff_degrees = left_wheel_degrees_curr - this->left_wheel_degrees_prev;
        double right_diff_degrees = right_wheel_degrees_curr - this->right_wheel_degrees_prev;
        
        poseChange.dxy_mm =  this->wheel_radius_mm * (radians(left_diff_degrees) + radians(right_diff_degrees));
        
        poseChange.dtheta_degrees = this->wheel_radius_mm / this->half_axle_length_mm *
        (right_diff_degrees - left_diff_degrees);
        
        poseChange.dt_seconds = timestamp_seconds_curr - timestamp_seconds_prev;
    }
    
    // Store current odometry for next time
    this->timestamp_seconds_prev = timestamp_seconds_curr;
    this->left_wheel_degrees_prev = left_wheel_degrees_curr;
    this->right_wheel_degrees_prev = right_wheel_degrees_curr;      
    
    return poseChange;
}
