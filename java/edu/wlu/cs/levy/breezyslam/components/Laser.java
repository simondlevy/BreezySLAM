/**
* 
* BreezySLAM: Simple, efficient SLAM in Java
*
* Laser.java - Java code for Laser model class
*
* Copyright (C) 2014 Simon D. Levy
*
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

package edu.wlu.cs.levy.breezyslam.components;

/**
* A class for scanning laser rangefinder (Lidar) parameters.
*/
public class Laser
{

   protected int scan_size;
   protected double scan_rate_hz;
   protected double detection_angle_degrees;
   protected double distance_no_detection_mm;
   protected int detection_margin;
   protected double offset_mm;
    
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
    public Laser(
        int scan_size,
        double scan_rate_hz,
        double detection_angle_degrees,
        double distance_no_detection_mm,
        int detection_margin,
        double offset_mm)
    {    
        this.scan_size = scan_size;
        this.scan_rate_hz = scan_rate_hz;
        this.detection_angle_degrees = detection_angle_degrees;
        this.distance_no_detection_mm = distance_no_detection_mm;
        this.detection_margin = detection_margin;
        this.offset_mm = offset_mm;
    }

    /**
    * Builds a Laser object by copying another Laser object.
    * Lidar unit.
    * @param laser                      the other Laser object
    * 
    */
    public Laser(Laser laser)
    {    
        this.scan_size = laser.scan_size;
        this.scan_rate_hz = laser.scan_rate_hz;
        this.detection_angle_degrees = laser.detection_angle_degrees;
        this.distance_no_detection_mm = laser.distance_no_detection_mm;
        this.detection_margin = laser.detection_margin;
        this.offset_mm = laser.offset_mm;
    }

    /**
     * Returns a string representation of this Laser object.
     */
      
    public String toString() 
    {
        String format = "scan_size=%d | scan_rate=%3.3f hz | " + 
                        "detection_angle=%3.3f deg | " + 
                        "distance_no_detection=%7.4f mm | " +
                        "detection_margin=%d | offset=%4.4f m";

        return String.format(format, this.scan_size,  this.scan_rate_hz,  
                             this.detection_angle_degrees, 
                             this.distance_no_detection_mm,  
                             this.detection_margin, 
                             this.offset_mm);
        
     }

    /**
     * Returns the offset of the laser in mm, from the center of the robot.
     *
     */
     public double getOffsetMm()
     {
         return this.offset_mm;
     }
}
