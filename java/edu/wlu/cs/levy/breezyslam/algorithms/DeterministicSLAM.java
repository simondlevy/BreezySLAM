/**
*    DeterministicSLAM implements SinglePositionSLAM using by returning the starting position instead of searching
*    on it; i.e., using odometry alone.
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
