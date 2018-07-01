/**
* 
* BreezySLAM: Simple, efficient SLAM in Java
*
* Map.java - Java code for Map class
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
* A class for maps used in SLAM.
*/
public class Map
{
	static 
    {
		System.loadLibrary("jnibreezyslam_components");
	}

	private native void init(int size_pixels, double size_meters);

    private double size_meters;

    private native void update(
            Scan scan, 
            double position_x_mm,   
            double position_y_mm,   
            double position_theta_degrees,
            int quality, 
            double hole_width_mm);
        
    private long native_ptr;

    /**
     * Returns a string representation of this Map object.
     */
    public native String toString();

    /**
     * Builds a square Map object.
     * @param size_pixels  size in pixels
     * @param size_meters  size in meters
     * 
     */
    public Map(int size_pixels, double size_meters)
    {
        this.init(size_pixels, size_meters);

        // for public accessor
        this.size_meters = size_meters;
    }

    /**
     * Puts current map values into bytearray, which should of which should be of 
     * this.size map_size_pixels ^ 2.
     * @param bytes byte array that gets the map values
     */
    public native void get(byte [] bytes);

    /**
     * Updates this map object based on new data.
     * @param scan a new scan
     * @param position a new postion
     * @param quality speed with which scan is integerate into map (0 through 255)
     * @param hole_width_mm hole width in millimeters
     * 
     */
    public void update(Scan scan, Position position, int quality, double hole_width_mm) 
    {
        this.update(scan, position.x_mm, position.y_mm, position.theta_degrees, quality, hole_width_mm);
    }

    /**
     * Returns the size of this map in meters.
     */
    public double sizeMeters()
    {
        return this.size_meters;
    }

}
