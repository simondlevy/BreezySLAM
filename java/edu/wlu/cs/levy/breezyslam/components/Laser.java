package edu.wlu.cs.levy.breezyslam.components;

public class Laser
{

   protected int scan_size;
   protected double scan_rate_hz;
   protected double detection_angle_degrees;
   protected double distance_no_detection_mm;
   protected int detection_margin;
   protected double offset_mm;
    
   public Laser(
        int scan_size,
        double scan_rate_hz,
        double detection_angle_degrees,
        double distance_no_detection_mm,
        int detection_margin,
        double offset_mm)
    {    
        this.scan_size = scan_size;
        this.scan_rate_hz = scan_rate_hz;
        this.detection_angle_degrees = detection_angle_degrees;
        this.distance_no_detection_mm = distance_no_detection_mm;
        this.detection_margin = detection_margin;
        this.offset_mm = offset_mm;
    }

   public Laser(Laser laser)
    {    
        this.scan_size = laser.scan_size;
        this.scan_rate_hz = laser.scan_rate_hz;
        this.detection_angle_degrees = laser.detection_angle_degrees;
        this.distance_no_detection_mm = laser.distance_no_detection_mm;
        this.detection_margin = laser.detection_margin;
        this.offset_mm = laser.offset_mm;
    }

    public String toString() 
    {
        String format = "scan_size=%d | scan_rate=%3.3f hz | " + 
                        "detection_angle=%3.3f deg | " + 
                        "distance_no_detection=%7.4f mm | " +
                        "detection_margin=%d | offset=%4.4f m";

        return String.format(format, this.scan_size,  this.scan_rate_hz,  
                             this.detection_angle_degrees, 
                             this.distance_no_detection_mm,  
                             this.detection_margin, 
                             this.offset_mm);
        
     }

     public double getOffsetMm()
     {
         return this.offset_mm;
     }
}
