/**
* 
* BreezySLAM: Simple, efficient SLAM in Java
*
* Scan.java - Java code for Scan class
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
* A class for Lidar scans.
*/
public class Scan
{
	static 
    {
		System.loadLibrary("jnibreezyslam_components");
	}

	private native void init(
            int span, 
            int scan_size,
            double scan_rate_hz,
            double detection_angle_degrees,
            double distance_no_detection_mm,
            int detection_margin,
            double offset_mm);
 
    private long native_ptr;

    public native String toString();

    /**
     * Returns a string representation of this Scan object.
     */
    public native void update(
            int [] lidar_mm,
            double hole_width_mm,
            double velocities_dxy_mm,
            double velocities_dtheta_degrees);


    /**
     * Builds a Scan object.
     * @param laser laser parameters
     * @param span supports spanning laser scan to cover the space better.
     * 
     */
    public Scan(Laser laser, int span)
    {
        this.init(span,
            laser.scan_size,
            laser.scan_rate_hz,                
            laser.detection_angle_degrees,     
            laser.distance_no_detection_mm,    
            laser.detection_margin,               
            laser.offset_mm);
    }

    /**
     * Builds a Scan object.
     * @param laser laser parameters
     * 
     */
    public Scan(Laser laser)
    {
        this(laser, 1);
    }

     /**
    * Updates this Scan object with new values from a Lidar scan.
    * @param scanvals_mm scanned Lidar distance values in millimeters
    * @param hole_width_millimeters hole width in millimeters
    * @param velocities forward velocity and angular velocity of robot at scan time
    * 
    */
    public void update(int [] scanvals_mm, double hole_width_millimeters, Velocities velocities) 
    {
        this.update(scanvals_mm, hole_width_millimeters, velocities.dxy_mm, velocities.dtheta_degrees);
    }
}

