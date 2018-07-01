/**
* 
* BreezySLAM: Simple, efficient SLAM in Java
*
* PoseChange.java - Java code for PoseChange class, encoding triple 
* (dxy_mm, dtheta_degrees, dt_seconds)
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
* A class representing the forward and angular velocities of a robot as determined by odometry.
*/
public class PoseChange 
{    
    
    /**
    * Creates a new PoseChange object with specified velocities.
    */
    public PoseChange(double dxy_mm, double dtheta_degrees, double dtSeconds)
    {
        this.dxy_mm = dxy_mm;
        this.dtheta_degrees = dtheta_degrees;
        this.dt_seconds = dtSeconds;
    }

    /**
    * Creates a new PoseChange object with zero velocities.
    */
    public PoseChange()
    {
        this.dxy_mm = 0;
        this.dtheta_degrees = 0;
        this.dt_seconds = 0;
    }

    /**
    * Updates this PoseChange object.
    * @param dxy_mm new forward distance traveled in millimeters
    * @param dtheta_degrees new angular rotation in degrees
    * @param dtSeconds time in seconds since last velocities
    */
    public void update(double dxy_mm, double dtheta_degrees, double dtSeconds)
    {
        double velocity_factor = (dtSeconds > 0) ?  (1 / dtSeconds) : 0;
        
        this.dxy_mm = dxy_mm * velocity_factor;
        
        this.dtheta_degrees = dtheta_degrees * velocity_factor;
    }

    /**
      * Returns a string representation of this PoseChange object.
      */
    public String toString()
    {
        return String.format("<dxy=%7.0f mm dtheta = %+3.3f degrees dt = %f s",
            this.dxy_mm, this.dtheta_degrees, this.dt_seconds);
    }

    /**
      * Returns the forward component of this PoseChange object.
      */
    public double getDxyMm()
    {
        return this.dxy_mm;
    }

    /**
      * Returns the angular component of this PoseChange object.
      */
     public double getDthetaDegrees()
    {
        return this.dtheta_degrees;
    }

    /**
      * Returns the time component of this PoseChange object.
      */
    public double getDtSeconds()
    {
        return this.dt_seconds;
    }

    /**
      * Forward component of velocity, in mm to be divided by time in seconds.
      */
    protected double dxy_mm;

    /**
      * Angular component of velocity, in mm to be divided by time in seconds.
      */

     protected double dtheta_degrees;

    /**
      *  Time in seconds between successive velocity measurements.
      */
     protected double dt_seconds;
}
