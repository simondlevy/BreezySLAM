/**
*    DeterministicSLAM implements SinglePositionSLAM using by returning the starting position instead of searching
*    on it; i.e., using odometry alone.
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
import edu.wlu.cs.levy.breezyslam.components.Laser;

public class DeterministicSLAM  extends SinglePositionSLAM
{

    /**
    * Creates a DeterministicSLAM object.
    * @param laser a Laser object containing parameters for your Lidar equipment
    * @param map_size_pixels the size of the desired map (map is square)
    * @param map_size_meters the size of the area to be mapped, in meters
    * @return a new CoreSLAM object
    */
    public DeterministicSLAM(Laser laser, int map_size_pixels, double map_size_meters)
    {
        super(laser, map_size_pixels, map_size_meters);
    }
    
    /**
    * Returns a new position identical to the starting position. Called automatically by
    * SinglePositionSLAM::updateMapAndPointcloud()
    * @param start_pos the starting position
    */
    protected Position getNewPosition(Position start_position)
    {
        return new Position(start_position.x_mm, start_position.y_mm, start_position.theta_degrees);
    }
     
} // DeterministicSLAM 
