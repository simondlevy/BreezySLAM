/**
*  RMHCSLAM implements SinglePositionSLAM using random-mutation hill-climbing search on the starting position.
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
import edu.wlu.cs.levy.breezyslam.components.PoseChange;
import edu.wlu.cs.levy.breezyslam.components.Map;
import edu.wlu.cs.levy.breezyslam.components.Scan;

public class RMHCSLAM extends SinglePositionSLAM
{
    private static final double DEFAULT_SIGMA_XY_MM         = 100;
    private static final double DEFAULT_SIGMA_THETA_DEGREES = 20;
    private static final int    DEFAULT_MAX_SEARCH_ITER     = 1000;

	static 
    {
		System.loadLibrary("jnibreezyslam_algorithms");
	}

	private native void init(int random_seed);

    private native Object positionSearch(
        Position start_pos,
        Map map,
        Scan scan,
        double sigma_xy_mm,
        double sigma_theta_degrees,
        int max_search_iter);


    private long native_ptr;

    /**
    * Creates an RMHCSLAM object.
    * @param laser a Laser object containing parameters for your Lidar equipment
    * @param map_size_pixels the size of the desired map (map is square)
    * @param map_size_meters the size of the area to be mapped, in meters
    * @param random_seed seed for psuedorandom number generator in particle filter
    * @return a new CoreSLAM object
    */
    public RMHCSLAM(Laser laser, int map_size_pixels, double map_size_meters, int random_seed)
    {
        super(laser, map_size_pixels, map_size_meters);

        this.init(random_seed);
    }
   
    /**
    * The standard deviation in millimeters of the Gaussian distribution of 
    * the (X,Y) component of position in the particle filter; default = 100
    */
    public double sigma_xy_mm = DEFAULT_SIGMA_XY_MM;;

    /**
    * The standard deviation in degrees of the Gaussian distribution of 
    * the angular rotation component of position in the particle filter; default = 20
    */
    public double sigma_theta_degrees = DEFAULT_SIGMA_THETA_DEGREES;   

    /**
    * The maximum number of iterations for particle-filter search; default = 1000
    */
    public int max_search_iter = DEFAULT_MAX_SEARCH_ITER;   

    /**
    * Returns a new position based on RMHC search from a starting position. Called automatically by
    * SinglePositionSLAM::updateMapAndPointcloud()
    * @param start_position the starting position
    */
    protected Position getNewPosition(Position start_position)
    {
        Position newpos = 
        (Position)positionSearch(
            start_position,
            this.map,
            this.scan_for_distance,
            this.sigma_xy_mm,
            this.sigma_theta_degrees,
            this.max_search_iter);    

        return newpos;
    }
   
} // RMHCSLAM
