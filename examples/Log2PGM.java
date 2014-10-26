/*
   Log2PGM.java : BreezySLAM demo.  Reads logfile with odometry and scan data from 
   Paris Mines Tech and produces a .PGM image file showing robot path 
   and final map.

   For details see

   @inproceedings{,
   author    = {Bruno Steux and Oussama El Hamzaoui},
   title     = {SinglePositionSLAM: a SLAM Algorithm in less than 200 lines of C code},
   booktitle = {11th International Conference on Control, Automation, Robotics and Vision, ICARCV 2010, Singapore, 7-10 
   December 2010, Proceedings},
   pages     = {1975-1979},
   publisher = {IEEE},
   year      = {2010}
   }

   Copyright (C) 2014 Simon D. Levy

   This code is free software: you can redistribute it and/or modify
   it under the terms of the GNU Lesser General Public License as 
   published by the Free Software Foundation, either version 3 of the 
   License, or (at your option) any later version.

   This code is distributed in the hope that it will be useful,     
   but WITHOUT ANY WARRANTY without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU Lesser General Public License 
   along with this code.  If not, see <http://www.gnu.org/licenses/>.
 */

import edu.wlu.cs.levy.breezyslam.components.Map;
import edu.wlu.cs.levy.breezyslam.components.Scan;
import edu.wlu.cs.levy.breezyslam.components.Position;
import edu.wlu.cs.levy.breezyslam.components.URG04LX;
import edu.wlu.cs.levy.breezyslam.components.Velocities;

import edu.wlu.cs.levy.breezyslam.robots.WheeledRobot;

import edu.wlu.cs.levy.breezyslam.algorithms.RMHCSLAM;
import edu.wlu.cs.levy.breezyslam.algorithms.DeterministicSLAM;
import edu.wlu.cs.levy.breezyslam.algorithms.SinglePositionSLAM;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Vector;

public class Log2PGM
{
    private static int    MAP_SIZE_PIXELS = 800;
    private static double MAP_SIZE_METERS = 32;
    private static int    SCAN_SIZE       = 682;

    private SinglePositionSLAM slam;

    private boolean use_odometry;
    private int random_seed;
    private Rover robot;

    static int coords2index(double x,  double y)
    {    
        return (int)(y * MAP_SIZE_PIXELS + x);
    }

    int mm2pix(double mm)
    {
        return (int)(mm / (MAP_SIZE_METERS * 1000. / MAP_SIZE_PIXELS));  
    }


    // Class for Mines verison of URG-04LX Lidar -----------------------------------

    private class MinesURG04LX extends URG04LX
    {
        public MinesURG04LX() 
        {
            super( 70,          // detectionMargin
                    145);         // offsetMillimeters
        }
    }

    // Class for MinesRover custom robot -------------------------------------------

    private class Rover extends WheeledRobot
    {
        public Rover() 
        {
            super(77,       // wheelRadiusMillimeters
                    165);     // halfAxleLengthMillimeters
        }    

        protected WheeledRobot.WheelOdometry extractOdometry(
                double timestamp, 
                double leftWheelOdometry, 
                double rightWheelOdometry)
        {        
            // Convert microseconds to seconds, ticks to angles        
            return new WheeledRobot.WheelOdometry(
                    timestamp / 1e6, 
                    ticksToDegrees(leftWheelOdometry),
                    ticksToDegrees(rightWheelOdometry));
        }

        protected String descriptorString()
        {
            return String.format("ticks_per_cycle=%d", this.TICKS_PER_CYCLE);
        }


        private double ticksToDegrees(double ticks)
        {
            return ticks * (180. / this.TICKS_PER_CYCLE);
        }

        private int TICKS_PER_CYCLE = 2000;
    }

    // Progress-bar class ----------------------------------------------------------------
    // Adapted from http://code.activestate.com/recipes/168639-progress-bar-class/
    // Downloaded 12 January 2014

    private class ProgressBar
    {
        public ProgressBar(int minValue, int maxValue, int totalWidth)
        {
            this.progBar = "[]";
            this.min = minValue;
            this.max = maxValue;
            this.span = maxValue - minValue;
            this.width = totalWidth;
            this.amount = 0;       // When amount == max, we are 100% done 
            this.updateAmount(0);  // Build progress bar string
        }

        void updateAmount(int newAmount)
        {
            if (newAmount < this.min)
            {
                newAmount = this.min;
            }
            if (newAmount > this.max)
            {
                newAmount = this.max;
            }

            this.amount = newAmount;

            // Figure out the new percent done, round to an integer
            float diffFromMin = (float)(this.amount - this.min);
            int percentDone = (int)java.lang.Math.round((diffFromMin / (float)this.span) * 100.0);

            // Figure out how many hash bars the percentage should be
            int allFull = this.width - 2;
            int numHashes = (int)java.lang.Math.round((percentDone / 100.0) * allFull);

            // Build a progress bar with hashes and spaces
            this.progBar = "[";
            this.addToProgBar("#", numHashes);
            this.addToProgBar(" ", allFull-numHashes);
            this.progBar += "]";

            // Figure out where to put the percentage, roughly centered
            int percentPlace =  (this.progBar.length() / 2) - ((int)(java.lang.Math.log10(percentDone+1)) + 1);
            String percentString = String.format("%d%%", percentDone);

            // Put it there
            this.progBar = this.progBar.substring(0,percentPlace) + percentString + this.progBar.substring(percentPlace+percentString.length());
        }

        String str()
        {   
            return this.progBar;
        }

        private String progBar; 
        private int min;
        private int max;
        private int span;
        private int width;
        private int amount;

        private void addToProgBar(String s, int n)
        {
            for (int k=0; k<n; ++k)
            {
                this.progBar += s;
            }
        }
    }

    public Log2PGM(boolean use_odometry, int random_seed)
    {
        MinesURG04LX laser = new MinesURG04LX();

        this.robot = new Rover(); 

        this.slam = random_seed > 0 ? 
            new RMHCSLAM(laser, MAP_SIZE_PIXELS, MAP_SIZE_METERS, random_seed) :
            new DeterministicSLAM(laser, MAP_SIZE_PIXELS, MAP_SIZE_METERS);

        this.use_odometry = use_odometry;
        this.random_seed = random_seed;
    }

    public byte [] processData(Vector<int []> scans, Vector<long []> odometries)
    {
        // Report what we're doing
        int nscans = scans.size();
        ProgressBar  progbar = new ProgressBar(0, nscans, 80); 

        System.out.printf("Processing %d scans with%s odometry / with%s particle filter...\n",
                nscans, this.use_odometry ? "" : "out", this.random_seed > 0 ? "" : "out");

        Vector<double []> trajectory = new Vector<double []>();

        for (int scanno=0; scanno<nscans; ++scanno)
        {
            int [] scan = scans.elementAt(scanno);

            // Update with/out odometry
            if (this.use_odometry)
            {
                long [] odometry = odometries.elementAt(scanno);
                Velocities velocities = this.robot.computeVelocities(odometry[0], odometry[1], odometry[2]);
                slam.update(scan, velocities);            
            }
            else
            {
                slam.update(scan);  
            }

            Position position = slam.getpos();

            // Add new coordinates to trajectory
            double [] v = new double[2];
            v[0] = position.x_mm;
            v[1] = position.y_mm;
            trajectory.add(v);     

 
            progbar.updateAmount(scanno);
            System.out.printf("\r%s", progbar.str());
        } 

        // Get final map
        byte [] mapbytes = new byte [MAP_SIZE_PIXELS*MAP_SIZE_PIXELS];
        slam.getmap(mapbytes);

        // Put trajectory into map as black pixels
        for (int k=0; k<trajectory.size(); ++k)
        {        
            double [] v = trajectory.elementAt(k);
                            
            int x = mm2pix(v[0]);
            int y = mm2pix(v[1]);
            
            mapbytes[coords2index(x, y)] = 0;
        }
     
        return mapbytes;
    }

    // main -------------------------------------------------------------------------------------------------------------

    public static void main(String[] argv)
    {
        // Bozo filter for input args
        if (argv.length < 3)
        {
            System.err.println("Usage:   java log2pgm <dataset> <use_odometry> <random_seed>");
            System.err.println("Example: java log2pgm exp2 1 9999");
            System.exit(1);
        }

        // Grab input args
        String dataset = argv[0];
        boolean use_odometry =  Integer.parseInt(argv[1])  == 0 ? false : true;
        int random_seed =  argv.length > 2 ? Integer.parseInt(argv[2]) : 0;

        // Load the data from the file    

        Vector<int []>  scans = new Vector<int []>();
        Vector<long []>  odometries = new Vector<long []>();;

        String filename = dataset + ".dat";
        System.out.printf("Loading data from %s ... \n", filename);

        BufferedReader input = null;

        try  
        {
            FileReader fstream = new FileReader(filename);
            input = new BufferedReader(fstream);
            while (true)
            {
                String line = input.readLine();
                if (line == null) 
                {
                    break;
                }

                String [] toks = line.split(" +");

                long [] odometry = new long [3];
                odometry[0] = Long.parseLong(toks[0]);
                odometry[1] = Long.parseLong(toks[2]);
                odometry[2] = Long.parseLong(toks[3]);
                odometries.add(odometry);

                int [] scan = new int [SCAN_SIZE];
                for (int k=0; k<SCAN_SIZE; ++k)
                {
                    scan[k] = Integer.parseInt(toks[k+24]);
                }
                scans.add(scan);
            }
            input.close();
        }
        catch (IOException e)
        {
            System.err.println("Error: " + e.getMessage());
        }

        // Create a Log2PGM object to process the data
        Log2PGM log2pgm = new Log2PGM(use_odometry, random_seed);

        // Process the scan and odometry data, returning a map
        byte [] mapbytes = log2pgm.processData(scans, odometries);

        // Save map and trajectory as PGM file    

        filename = dataset + ".pgm";
        System.out.println("\nSaving map to file " + filename);

        BufferedWriter output = null;

        try  
        {
            FileWriter fstream = new FileWriter(filename);
            output = new BufferedWriter(fstream);
            output.write(String.format("P2\n%d %d 255\n", MAP_SIZE_PIXELS, MAP_SIZE_PIXELS));
            for (int y=0; y<MAP_SIZE_PIXELS; y++)
            {
                for (int x=0; x<MAP_SIZE_PIXELS; x++)
                {
                    // Output unsigned byte value
                    output.write(String.format("%d ", (int)mapbytes[coords2index(x, y)] & 0xFF));
                }
                output.write("\n");
            }
            output.close();
        }
        catch (IOException e)
        {
            System.err.println("Error: " + e.getMessage());
        }
    }
}
