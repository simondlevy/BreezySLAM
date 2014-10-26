package edu.wlu.cs.levy.breezyslam.algorithms;

import edu.wlu.cs.levy.breezyslam.components.Laser;
import edu.wlu.cs.levy.breezyslam.components.Velocities;

public class CoreSLAM {
    
    public CoreSLAM(Laser laser, int map_size_pixels, double map_size_meters)
    {
        /*
        // Set default params
        this->map_quality = DEFAULT_MAP_QUALITY;
        this->hole_width_mm = DEFAULT_HOLE_WIDTH_MM;   
        
        // Store laser for later
        this->laser = new Laser(laser);
        
        // Initialize velocities (dxyMillimeters, dthetaDegrees, dtSeconds) for odometry
        this->velocities = new Velocities();

        // Initialize a scan for computing distance to map, and one for updating map
        this->scan_for_mapbuild = this->scan_create(3);
        this->scan_for_distance = this->scan_create(1);
        
        // Initialize the map 
        this->map = new Map(map_size_pixels, map_size_meters);
        */
    }


    public void update(int [] scan_mm, Velocities velocities)
    {             
        /*
        // Build a scan for computing distance to map, and one for updating map
        this->scan_update(this->scan_for_mapbuild, scan_mm);
        this->scan_update(this->scan_for_distance, scan_mm);
        
        // Update velocities
        this->velocities->update(velocities.dxy_mm, 
                                 velocities.dtheta_degrees,  
                                 velocities.dt_seconds);
                                 
        // Implementing class updates map and pointcloud
        this->updateMapAndPointcloud(velocities);
        */
    }   


    public void getmap(byte [] mapbytes)
    {
        //this->map->get((char *)mapbytes);
    }

    /*
    Scan * CoreSLAM::scan_create(int span)
    {
        //return new Scan(this->laser, span);
    }


    void scan_update(Scan scan, int [] scan_mm)
    {
        //scan->update(scan_mm, this->hole_width_mm, *this->velocities);
    }
    */
}
