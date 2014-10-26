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

protected:

    int scan_size;                      /* number of points per scan */
    double scan_rate_hz;                /* scans per second */
    double detection_angle_degrees;     /* e.g. 240, 360 */
    double distance_no_detection_mm;    /* default value when the laser returns 0 */
    int detection_margin;               /* first scan element to consider */
    double offset_mm;                   /* position of the laser wrt center of rotation */
    
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
        )

    {
        this->scan_size = scan_size;
        this->scan_rate_hz = scan_rate_hz;
        this->detection_angle_degrees = detection_angle_degrees;
        this->distance_no_detection_mm = distance_no_detection_mm;
        this->detection_margin = detection_margin;
        this->offset_mm = offset_mm;
    }
    
    /**
    * Builds an empty Laser object (all parameters zero).
    */
    Laser(void);
    
    friend ostream& operator<< (ostream & out, Laser & laser) 
    {
        char str[512];

        sprintf(str, 
             "<scan_size=%d | scan_rate=%3.3f hz | " 
             "detection_angle=%3.3f deg | " 
             "distance_no_detection=%7.4f mm | " 
             "detection_margin=%d | offset=%4.4f mm>",
             laser.scan_size,  laser.scan_rate_hz,  
             laser.detection_angle_degrees, 
             laser.distance_no_detection_mm,  
             laser.detection_margin, 
             laser.offset_mm);

        out << str;

        return out;
    }
};
    
/**
  * A class for the Hokuyo URG-04LX laser.
  */
class URG04LX : public Laser  
{
    
public:
    
    /**
    * Builds a URG04LX object.
    * Lidar unit.
    * @param detection_margin           number of rays at edges of scan to ignore
    * @param offset_mm                  forward/backward offset of laser motor from robot center
    * @return a new URG04LX object
    * 
    */
    URG04LX(int detection_margin = 0, float offset_mm = 0) : 
    Laser(682, 10, 240, 4000, detection_margin, offset_mm) { }
    
    /**
    * Builds an empty URG04LX object (all parameters zero).
    */
    URG04LX(void) : Laser() {} 
    
};

