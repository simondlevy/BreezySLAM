package edu.wlu.cs.levy.breezyslam.components;

public class Position
{

   public double x_mm;
   public double y_mm;
   public double theta_degrees;
    
   public Position(double x_mm, double y_mm, double theta_degrees)
    {    
        this.x_mm = x_mm;
        this.y_mm = y_mm;
        this.theta_degrees = theta_degrees;
    }

   public Position(Position position)
    {    
        this.x_mm = position.x_mm;
        this.y_mm = position.y_mm;
        this.theta_degrees = position.theta_degrees;
     }

    public String toString() 
    {
        //String format = "<x = %7.0f mm  y = %7.0f mm theta = %+3.3f degrees>";
        String format = "<x = %f mm  y = %f mm theta = %f degrees>";

        return String.format(format, this.x_mm, this.y_mm, this.theta_degrees);
        
     }

 	public static void main(String[] argv)
	{
        Position position = new Position(300, 400, 120);
		System.out.println(position);
	}
}
