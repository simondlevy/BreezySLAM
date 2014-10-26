/**
* SinglePositionSLAM is an abstract class that implements CoreSLAM using a point-cloud
* with a single point (particle, position). Implementing classes should provide the method
*    
*    Position getNewPosition(Position start_position)
*      
* to compute a new position based on searching from a starting position.
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

package edu.wlu.cs.levy.breezyslam.algorithms;

import edu.wlu.cs.levy.breezyslam.components.Position;
import edu.wlu.cs.levy.breezyslam.components.Velocities;
import edu.wlu.cs.levy.breezyslam.components.Laser;


public abstract class SinglePositionSLAM extends CoreSLAM
{
    /**
    * Returns the current position.
    * @return the current position as a Position object.
    */
    public Position getpos()
    {
        return this.position;
    }

    /**
    * Creates a SinglePositionSLAM object.
    * @param laser a Laser object containing parameters for your Lidar equipment
    * @param map_size_pixels the size of the desired map (map is square)
    * @param map_size_meters the size of the area to be mapped, in meters
    * @return a new SinglePositionSLAM object
    */
    protected SinglePositionSLAM(Laser laser, int map_size_pixels, double map_size_meters)
    {
        super(laser, map_size_pixels, map_size_meters);

        this.position = new Position(this.init_coord_mm(), this.init_coord_mm(), 0);
    }
    
    
    /**
    * Updates the map and point-cloud (particle cloud). Called automatically by CoreSLAM::update()
    * @param velocities velocities for odometry
    */
    protected void updateMapAndPointcloud(Velocities velocities)
    {
        // Start at current position 
        Position start_pos = new Position(this.position);

        // Add effect of velocities
        start_pos.x_mm      += velocities.getDxyMm() * this.costheta();
        start_pos.y_mm      += velocities.getDxyMm() *  this.sintheta();
        start_pos.theta_degrees += velocities.getDthetaDegrees();
        
        // Add offset from laser
        start_pos.x_mm += this.laser.getOffsetMm() * this.costheta();
        start_pos.y_mm += this.laser.getOffsetMm() * this.sintheta();
        
        // Get new position from implementing class
        Position new_position = this.getNewPosition(start_pos);
             
        // Update the map with this new position
        this.map.update(this.scan_for_mapbuild, new_position, this.map_quality, this.hole_width_mm);
       
        // Update the current position with this new position, adjusted by laser offset
        this.position = new Position(new_position);
        this.position.x_mm -= this.laser.getOffsetMm() * this.costheta();
        this.position.y_mm -= this.laser.getOffsetMm() * this.sintheta(); 
    } 

    /**
    * Returns a new position based on searching from a starting position. Called automatically by
    * SinglePositionSLAM::updateMapAndPointcloud()
    * @param start_pos the starting position
    */

    protected abstract Position getNewPosition(Position start_pos);    
    
    private Position position;
       
    private double theta_radians()
    {
        return java.lang.Math.toRadians(this.position.theta_degrees);
    }
    
    private double costheta()
    {
        return java.lang.Math.cos(this.theta_radians());
    }
    
    private double sintheta()
    {
        return java.lang.Math.sin(this.theta_radians());
    }
    
    private double init_coord_mm()
    {
        // Center of map
        return 500 * this.map.sizeMeters();
    }
}
