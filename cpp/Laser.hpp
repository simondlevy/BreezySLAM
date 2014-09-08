/**
*
* Laser.hpp - C++ headers for Laser model classes
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
* A class for scanning laser rangefinder (Lidar) parameters.
*/
class Laser  
{
    friend class CoreSLAM;
    friend class SinglePositionSLAM;
    friend class RMHC_SLAM;
    friend class Scan;
    
public:
    
    /**
    * Builds a Laser object from parameters based on the specifications for your 
    * Lidar unit.
    * @param scan_size                  number of rays per scan
    * @param scan_rate_hz               laser scan rate in Hertz
    * @param detection_angle_degrees    detection angle in degrees (e.g. 240, 360)
    * @param detection_margin           number of rays at edges of scan to ignore
    * @param offset_mm                  forward/backward offset of laser motor from robot center
    * @return a new Laser object
    * 
    */
    Laser(
        int scan_size,
        float scan_rate_hz,
        float detection_angle_degrees,
        float distance_no_detection_mm,
        int detection_margin = 0,
        float offset_mm = 0.
        );
    
    /**
    * Builds an empty Laser object (all parameters zero).
    */
    Laser(void);
    
    /**
    * Dealloates memory for this Laser object.
    */
    ~Laser(void);
    
    
    friend ostream& operator<< (ostream & out, Laser & laser);

private:
    
    struct laser_t * laser;
    
};
    
class URG04LX : public Laser  
{
    
public:
    
    /**
    * Builds a Laser object from parameters based on the specifications for your 
    * Lidar unit.
    * @param detection_margin           number of rays at edges of scan to ignore
    * @param offset_mm                  forward/backward offset of laser motor from robot center
    * @return a new URG04LX object
    * 
    */
    URG04LX(int detection_margin = 0, float offset_mm = 0) : 
    Laser(682, 10, 240, 4000, detection_margin, offset_mm) { }
    
    /**
    * Builds an empty Laser object (all parameters zero).
    */
    URG04LX(void) : Laser() {} 
    
};

