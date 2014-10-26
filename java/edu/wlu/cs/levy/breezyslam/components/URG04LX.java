package edu.wlu.cs.levy.breezyslam.components;

public class URG04LX extends Laser
{

   public URG04LX(int detection_margin, double offset_mm)
    {    
        super(682, 10, 240, 4000, detection_margin, offset_mm);
    }

   public URG04LX()
    {    
        this(0, 0);
    }
}
