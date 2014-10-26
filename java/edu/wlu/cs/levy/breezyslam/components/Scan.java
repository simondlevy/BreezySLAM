package edu.wlu.cs.levy.breezyslam.components;

public class Scan
{
	static 
    {
		System.loadLibrary("jnibreezyslam_components");
	}

	private native void init(
            int span, 
            int scan_size,
            double scan_rate_hz,
            double detection_angle_degrees,
            double distance_no_detection_mm,
            int detection_margin,
            double offset_mm);
 
    private long native_ptr;

    public native String toString();

    public native void update(
            int [] lidar_mm,
            double hole_width_mm,
            double velocities_dxy_mm,
            double velocities_dtheta_degrees);


    public Scan(Laser laser, int span)
    {
        this.init(span,
            laser.scan_size,
            laser.scan_rate_hz,                
            laser.detection_angle_degrees,     
            laser.distance_no_detection_mm,    
            laser.detection_margin,               
            laser.offset_mm);
    }

    public Scan(Laser laser)
    {
        this(laser, 1);
    }

     /**
    * Updates this Scan object with new values from a Lidar scan.
    * @param scanvals_mm scanned Lidar distance values in millimeters
    * @param hole_width_millimeters hole width in millimeters
    * @param velocities forward velocity and angular velocity of robot at scan time
    * 
    */
    public void update(int [] scanvals_mm, double hole_width_millimeters, Velocities velocities) 
    {
        this.update(scanvals_mm, hole_width_millimeters, velocities.dxy_mm, velocities.dtheta_degrees);
    }
}

