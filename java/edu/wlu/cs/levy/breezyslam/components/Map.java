package edu.wlu.cs.levy.breezyslam.components;

public class Map
{
	static 
    {
		System.loadLibrary("jnibreezyslam_components");
	}

	private native void init(int size_pixels, double size_meters);

    private double size_meters;

    private native void update(
            Scan scan, 
            double position_x_mm,   
            double position_y_mm,   
            double position_theta_degrees,
            int quality, 
            double hole_width_mm);
        
    private long native_ptr;

    public native String toString();

    public Map(int size_pixels, double size_meters)
    {
        this.init(size_pixels, size_meters);

        // for public accessor
        this.size_meters = size_meters;
    }

    /**
     * Puts current map values into bytearray, which should of which should be of 
     * this->size map_size_pixels ^ 2.
     */
    public native void get(byte [] bytes);

    /**
     * Updates this map object based on new data.
     * @param scan a new scan
     * @param position a new postion
     * @param quality speed with which scan is integerate into map (0 through 255)
     * @param hole_width_mm hole width in millimeters
     * 
     */
    public void update(Scan scan, Position position, int quality, double hole_width_mm) 
    {
        this.update(scan, position.x_mm, position.y_mm, position.theta_degrees, quality, hole_width_mm);
    }

    public double sizeMeters()
    {
        return this.size_meters;
    }

}
