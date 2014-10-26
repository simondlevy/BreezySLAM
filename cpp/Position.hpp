/**
* 
* Position.hpp - C++ header for Position class
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
* A class representing the position of a robot.
*/

class Position 
{    
    friend class CoreSLAM;
    
public:
    
    /**
    * Constructs a new position.
    * @param x_mm X coordinate in millimeters
    * @param y_mm Y coordinate in millimeters
    * @param theta_degrees rotation angle in degrees
    */
    
    Position(double x_mm, double y_mm, double theta_degrees)
    {
        this->x_mm = x_mm;
        this->y_mm = y_mm;
        this->theta_degrees = theta_degrees;
        
    }
    
    /**
    * Empty position constructor.  All values are set to zero.
    */
    Position(void)
    {
        this->x_mm = 0;
        this->y_mm = 0;
        this->theta_degrees = 0;       
    }
    
    
    friend ostream& operator<< (ostream & out, Position & position)
    {
        char str[100];
        
        //sprintf(str, "<x = %7.0f mm  y = %7.0f mm theta = %+3.3f degrees>",
        sprintf(str, "<x = %f mm  y = %f mm theta = %f degrees>",
            position.x_mm, position.y_mm, position.theta_degrees);
        
        out << str;
        
        return out;
    }
    
    /**
    * Distance of robot from left edge of map, in millimeters.
    */
    double x_mm;
    
    
    /**
    * Distance of robot from top edge of map, in millimeters.
    */
    double y_mm;
    
    /**
    * Clockwise rotation of robot with respect to three o'clock (east), in degrees.
    */
    double theta_degrees;    
};
