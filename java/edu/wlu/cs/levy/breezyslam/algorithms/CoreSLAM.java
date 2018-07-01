/**
*
* CoreSLAM.java abstract Java class for CoreSLAM algorithm in BreezySLAM
*
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


package edu.wlu.cs.levy.breezyslam.algorithms;

import edu.wlu.cs.levy.breezyslam.components.Laser;
import edu.wlu.cs.levy.breezyslam.components.PoseChange;
import edu.wlu.cs.levy.breezyslam.components.Map;
import edu.wlu.cs.levy.breezyslam.components.Scan;

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
public abstract class CoreSLAM {

    /**
    * The quality of the map (0 through 255)
    */
    public int map_quality = 50; 

    /**
    * The width in millimeters of each "hole" in the map (essentially, wall width)
    */
    public double hole_width_mm = 600;

    protected Laser laser;

    protected PoseChange poseChange;

    protected Map map;

    protected Scan scan_for_mapbuild;
    protected Scan scan_for_distance;

    public CoreSLAM(Laser laser, int map_size_pixels, double map_size_meters)
    {
        // Set default params
        this.laser = new Laser(laser);
        
        // Initialize poseChange (dxyMillimeters, dthetaDegrees, dtSeconds) for odometry
        this.poseChange = new PoseChange();

        // Initialize a scan for computing distance to map, and one for updating map
        this.scan_for_mapbuild = this.scan_create(3);
        this.scan_for_distance = this.scan_create(1);
        
        // Initialize the map 
        this.map = new Map(map_size_pixels, map_size_meters);
    }

    private Scan scan_create(int span)
    {
        return new Scan(this.laser, span);
    }

    private void scan_update(Scan scan, int [] scan_mm)
    {
        scan.update(scan_mm, this.hole_width_mm, this.poseChange);
    }


    public void update(int [] scan_mm, PoseChange poseChange)
    {             
        // Build a scan for computing distance to map, and one for updating map
        this.scan_update(this.scan_for_mapbuild, scan_mm);
        this.scan_update(this.scan_for_distance, scan_mm);
        
        // Update poseChange
        this.poseChange.update(poseChange.getDxyMm(), poseChange.getDthetaDegrees(),  poseChange.getDtSeconds());
                                 
        // Implementing class updates map and pointcloud
        this.updateMapAndPointcloud(poseChange);
    }   

    /**
    * Updates the scan, and calls the the implementing class's updateMapAndPointcloud method with zero poseChange
    * (no odometry).
    * @param scan_mm Lidar scan values, whose count is specified in the <tt>scan_size</tt> 
    * attribute of the Laser object passed to the CoreSlam constructor
    */
    public void update(int [] scan_mm)
    {
        PoseChange zero_poseChange = new PoseChange();

        this.update(scan_mm, zero_poseChange);
    }


    protected abstract void updateMapAndPointcloud(PoseChange poseChange);

    public void getmap(byte [] mapbytes)
    {
        this.map.get(mapbytes);
    }

}
