/**
* 
* BreezySLAM: Simple, efficient SLAM in C++
*
* WheeledRobot.hpp - C++ header for WheeledRobot class
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

class PoseChange;

/**
* An abstract class for wheeled robots.  Supports computation of forward and angular 
* poseChange based on odometry. Your subclass should should implement the
* extractOdometry method.
*/
class WheeledRobot 
{
    
protected:
    
/**
* Builds a WheeledRobot object. Parameters should be based on the specifications for
* your robot.
* @param wheel_radius_mm radius of each odometry wheel, in meters        
* @param half_axle_length_mm half the length of the axle between the odometry wheels, in meters  
* @return a new WheeledRobot object
*/
WheeledRobot(double wheel_radius_mm, double half_axle_length_mm)
{
    this->wheel_radius_mm = wheel_radius_mm;     
    this->half_axle_length_mm = half_axle_length_mm;
    
    this->timestamp_seconds_prev = 0;
    this->left_wheel_degrees_prev = 0;
    this->right_wheel_degrees_prev = 0;
}

/**
* Computes forward and angular poseChange based on odometry.
* @param timestamp time stamp, in whatever units your robot uses       
* @param left_wheel_odometry odometry for left wheel, in whatever units your robot uses       
* @param right_wheel_odometry odometry for right wheel, in whatever units your robot uses
* @return poseChange object representing poseChange for these odometry values
*/

PoseChange
computePoseChange(
    double timestamp, 
    double left_wheel_odometry, 
    double right_wheel_odometry);

/**
* Extracts usable odometry values from your robot's odometry.
* @param timestamp time stamp, in whatever units your robot uses       
* @param left_wheel_odometry odometry for left wheel, in whatever units your robot uses       
* @param right_wheel_odometry odometry for right wheel, in whatever units your robot uses
* @param timestamp_seconds gets time stamp in seconds
* @param left_wheel_degrees gets left wheel rotation in degrees
* @param right_wheel_degrees gets right wheel rotation in degrees
*/
virtual void extractOdometry(
    double timestamp, 
    double left_wheel_odometry, 
    double right_wheel_odometry, 
    double & timestamp_seconds, 
    double & left_wheel_degrees, 
    double & right_wheel_degrees) = 0;

friend ostream& operator<< (ostream & out, WheeledRobot & robot)
{
    
    char subclassStr[100];
    robot.descriptorString(subclassStr);
    
    char str[200];
    sprintf(str, "<Wheel radius=%f m Half axle Length=%f m | %s>",
        robot.wheel_radius_mm, robot.half_axle_length_mm, subclassStr);
    
    out << str;
    
    return out;
}


/**
* Gets a descriptor string for your robot.
* @param str gets the descriptor string    
*/
virtual void descriptorString(char * str) = 0;

private:
    
    double wheel_radius_mm;
    double half_axle_length_mm;
    
    double timestamp_seconds_prev;
    double left_wheel_degrees_prev;
    double right_wheel_degrees_prev;    
    
    static double radians(double degrees)
    {
        return degrees * M_PI / 180;
    }
    
    
};        
