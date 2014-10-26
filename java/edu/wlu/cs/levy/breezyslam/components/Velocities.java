package edu.wlu.cs.levy.breezyslam.components;

/**
* A class representing the forward and angular velocities of a robot.
*/
public class Velocities 
{    
    
    /**
    * Creates a new Velocities object with specified velocities.
    */
    public Velocities(double dxy_mm, double dtheta_degrees, double dtSeconds)
    {
        this.dxy_mm = dxy_mm;
        this.dtheta_degrees = dtheta_degrees;
        this.dt_seconds = dtSeconds;
    }

    /**
    * Creates a new Velocities object with zero velocities.
    */
    public Velocities()
    {
        this.dxy_mm = 0;
        this.dtheta_degrees = 0;
        this.dt_seconds = 0;
    }

    /**
    * Updates this Velocities object.
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

    public String toString()
    {
        return String.format("<dxy=%7.0f mm dtheta = %+3.3f degrees dt = %f s",
            this.dxy_mm, this.dtheta_degrees, this.dt_seconds);
    }

    public double getDxyMm()
    {
        return this.dxy_mm;
    }

    public double getDthetaDegrees()
    {
        return this.dtheta_degrees;
    }

    public double getDtSeconds()
    {
        return this.dt_seconds;
    }

    protected double dxy_mm;
    protected double dtheta_degrees;
    protected double dt_seconds;
}
