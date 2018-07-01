/**
 * 
 * BreezySLAM: Simple, efficient SLAM in Java
 *
 * Position.java - Java code for Position class
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
 * A class representing the position of a robot.
 */
public class Position
{

    /**
     * Distance of robot from left edge of map, in millimeters.
     */
    public double x_mm;

    /**
     * Distance of robot from top edge of map, in millimeters.
     */
    public double y_mm;

    /**
     * Clockwise rotation of robot with respect to three o'clock (east), in degrees.
     */
    public double theta_degrees;    

    /**
    * Constructs a new position.
    * @param x_mm X coordinate in millimeters
    * @param y_mm Y coordinate in millimeters
    * @param theta_degrees rotation angle in degrees
    */
     public Position(double x_mm, double y_mm, double theta_degrees)
    {    
        this.x_mm = x_mm;
        this.y_mm = y_mm;
        this.theta_degrees = theta_degrees;
    }

    /**
      * Constructs a new Position object by copying another.
      * @param the other Positon object
      */
    
    public Position(Position position)
    {    
        this.x_mm = position.x_mm;
        this.y_mm = position.y_mm;
        this.theta_degrees = position.theta_degrees;
    }

    /**
      * Returns a string representation of this Position object.
      */
    public String toString() 
    {
        String format = "<x = %7.0f mm  y = %7.0f mm theta = %+3.3f degrees>";

        return String.format(format, this.x_mm, this.y_mm, this.theta_degrees);

    }
}
