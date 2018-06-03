/**
*
* algorithms.hpp - C++ header for SLAM algorithms.
**
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

class Position;
class Map;
class Scan;
class Laser;

/**
*    CoreSLAM is an abstract class that uses the classes Position, Map, Scan, and Laser
*    to run variants of the simple CoreSLAM (tinySLAM) algorithm described in 
*    <pre>
*     @inproceedings{coreslam-2010,
*       author    = {Bruno Steux and Oussama El Hamzaoui}, 
*       title     = {CoreSLAM: a SLAM Algorithm in less than 200 lines of C code},  
*       booktitle = {11th International Conference on Control, Automation,   
*                    Robotics and Vision, ICARCV 2010, Singapore, 7-10  
*                    December 2010, Proceedings},  
*       pages     = {1975-1979},  
*       publisher = {IEEE},  
*       year      = {2010}
*     }
*     </pre>
*    Implementing classes should provide the method
*
*      void updateMapAndPointcloud(int * scan_mm, PoseChange & poseChange)
*
*    to update the map and point-cloud (particle cloud).
*
*/
class CoreSLAM 
{

public:
    
    /**
    * Computes distance between a scan and map, given hypothetical position, to support particle filtering.
    * @param scan the scan
    * @param map the map
    * @param position the position
    * @return distance in arbitrary units, or -1 for infinity
    */
    static int distanceScanToMap(Scan & scan, Map & map, Position & position);
    
    /**
    * Retrieves the current map.
    * @param mapbytes a byte array big enough to hold the map (map_size_pixels * map_size_pixels)
    */
    void getmap(unsigned char * mapbytes);
    
   /**
    * Updates the scan and odometry, and calls the the implementing class's updateMapAndPointcloud method with
    * the specified poseChange.
    * 
    * @param scan_mm Lidar scan values, whose count is specified in the <tt>scan_size</tt> 
    * attribute of the Laser object passed to the CoreSlam constructor
    * @param poseChange poseChange for odometry
    */
    void update(int * scan_mm, PoseChange & poseChange);


    /**
    * Updates the scan, and calls the the implementing class's updateMapAndPointcloud method with zero poseChange
    * (no odometry).
    * @param scan_mm Lidar scan values, whose count is specified in the <tt>scan_size</tt> 
    * attribute of the Laser object passed to the CoreSlam constructor
    */
    void update(int * scan_mm);
    
    /**
    * The quality of the map (0 through 255); default = 50
    */
    int map_quality;

    /**
    * The width in millimeters of each "hole" in the map (essentially, wall width); 
    * default = 600
    */
    double hole_width_mm;

protected:

    /**
    * Creates a CoreSLAM object.
    * @param laser a Laser object containing parameters for your Lidar equipment
    * @param map_size_pixels the size of the desired map (map is square)
    * @param map_size_meters the size of the area to be mapped, in meters
    * @return a new CoreSLAM object
    */
    CoreSLAM(Laser & laser, int map_size_pixels, double map_size_meters);

    /**
    * Deallocates this CoreSLAM object.
    */
    ~CoreSLAM(void);

     /**
     * A pointer to the current map
     */
    Map * map;
    
    /**
    * A pointer to the laser model
    */
    Laser * laser;
    
    /**
    * A scan for building the map
    */
    Scan * scan_for_mapbuild;
    
    /**
    * A scan for use in distanceScanToMap()
    */
    Scan * scan_for_distance;    
    
    /**
    * The current poseChange from odometry
    */
    class PoseChange * poseChange;
        
    /**
    * Updates the map and point-cloud (particle cloud). Called automatically by CoreSLAM::update()
    * @param poseChange poseChange for odometry
    */
    virtual void updateMapAndPointcloud(PoseChange & poseChange) = 0;

private:
            
    Scan * scan_create(int span);
    
    void scan_update(Scan * scan, int * scan_mm);
   
}; // CoreSLAM


/**
*    SinglePositionSLAM is an abstract class that implements CoreSLAM using a point-cloud
*    with a single point (particle, position). Implementing classes should provide the method
*    
*      Position & getNewPosition(Position & start_position)
*      
*    to compute a new position based on searching from a starting position.
*/
class SinglePositionSLAM : public CoreSLAM
{

public:

    /**
    * Returns the current position.
    * @return the current position as a Position object.
    */
    Position & getpos(void);

protected:

    /**
    * Creates a SinglePositionSLAM object.
    * @param laser a Laser object containing parameters for your Lidar equipment
    * @param map_size_pixels the size of the desired map (map is square)
    * @param map_size_meters the size of the area to be mapped, in meters
    * @return a new SinglePositionSLAM object
    */
    SinglePositionSLAM(Laser & laser, int map_size_pixels, double map_size_meters);
    
    
    /**
    * Updates the map and point-cloud (particle cloud). Called automatically by CoreSLAM::update()
    * @param poseChange poseChange for odometry
    */
    void updateMapAndPointcloud(PoseChange & poseChange);
    
    /**
    * Returns a new position based on searching from a starting position. Called automatically by
    * SinglePositionSLAM::updateMapAndPointcloud()
    * @param start_pos the starting position
    */
    virtual Position getNewPosition(Position & start_pos) = 0;    
    
    
private:    
    
    Position position;
       
    double init_coord_mm(void);
    
    double theta_radians(void);
    
    double costheta(void);
    
    double sintheta(void);    
    
};

/**
*    RMHC_SLAM implements SinglePositionSLAM using random-mutation hill-climbing search on the starting position.
*/
class RMHC_SLAM : public SinglePositionSLAM
{

public:

    /**
    * Creates an RMHC_SLAM object.
    * @param laser a Laser object containing parameters for your Lidar equipment
    * @param map_size_pixels the size of the desired map (map is square)
    * @param map_size_meters the size of the area to be mapped, in meters
    * @param random_seed seed for psuedorandom number generator in particle filter
    * @return a new CoreSLAM object
    */
    RMHC_SLAM(Laser & laser, 
        int map_size_pixels,
        double map_size_meters, 
        unsigned random_seed);

    ~RMHC_SLAM(void);    
    
   
    /**
    * The standard deviation in millimeters of the Gaussian distribution of 
    * the (X,Y) component of position in the particle filter; default = 100
    */
    double sigma_xy_mm;

    /**
    * The standard deviation in degrees of the Gaussian distribution of 
    * the angular rotation component of position in the particle filter; default = 20
    */
    double sigma_theta_degrees;   

    /**
    * The maximum number of iterations for particle-filter search; default = 1000
    */
    int max_search_iter;   

protected:

    /**
    * Returns a new position based on RMHC search from a starting position. Called automatically by
    * SinglePositionSLAM::updateMapAndPointcloud()
    * @param start_position the starting position
    */
    Position getNewPosition(Position & start_position) ;

private:

    // Pseudorandom-number generator
    void * randomizer;
   
}; // RMHC_SLAM

/**
*    Deterministic_SLAM implements SinglePositionSLAM using by returning the starting position instead of searching
*    on it; i.e., using odometry alone.
*/
class Deterministic_SLAM : public SinglePositionSLAM
{

public:

    /**
    * Creates a Deterministic_SLAM object.
    * @param laser a Laser object containing parameters for your Lidar equipment
    * @param map_size_pixels the size of the desired map (map is square)
    * @param map_size_meters the size of the area to be mapped, in meters
    * @return a new CoreSLAM object
    */
    Deterministic_SLAM(Laser & laser, int map_size_pixels, double map_size_meters);
    
protected:

    /**
    * Returns a new position identical to the starting position. Called automatically by
    * SinglePositionSLAM::updateMapAndPointcloud()
    * @param start_pos the starting position
    */
    Position getNewPosition(Position & start_position) ;
     
}; // Deterministic_SLAM 
