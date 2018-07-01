package edu.wlu.cs.levy.breezyslam.components;

/**
  * A class for the Hokuyo URG-04LX laser.
  */
public class URG04LX extends Laser
{

    /**
    * Builds a URG04LX object.
    * Lidar unit.
    * @param detection_margin           number of rays at edges of scan to ignore
    * @param offset_mm                  forward/backward offset of laser motor from robot center
    * @return a new URG04LX object
    * 
    */
    public URG04LX(int detection_margin, double offset_mm)
    {    
        super(682, 10, 240, 4000, detection_margin, offset_mm);
    }

    /**
      * Builds a URG04LX object with zero detection margin, offset mm.
      */
   public URG04LX()
    {    
        this(0, 0);
    }
}
