/*
log2pgm.cpp : BreezySLAM demo.  Reads logfile with odometry and scan data from 
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

Change log:

20-APR-2014 - Simon D. Levy - Get params from command line
05-JUN-2014 - SDL - get random seed from command line
*/

// SinglePositionSLAM params: gives us a nice-size map
static const int MAP_SIZE_PIXELS        = 800;
static const double MAP_SIZE_METERS     =  32;

static const int SCAN_SIZE 		        = 682;

// Arbitrary maximum length of line in input logfile
#define MAXLINE 10000

#include <iostream>
#include <vector>
using namespace std;

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <time.h>

#include "Position.hpp"
#include "Laser.hpp"
#include "WheeledRobot.hpp"
#include "PoseChange.hpp"
#include "algorithms.hpp"


// Methods to load all data from file ------------------------------------------
// Each line in the file has the format:
//
//  TIMESTAMP  ... Q1  Q1 ... Distances
//  (usec)                    (mm)
//  0          ... 2   3  ... 24 ... 
//  
//where Q1, Q2 are odometry values

static void skiptok(char ** cpp)
{
    *cpp = strtok(NULL, " ");
}

static int nextint(char ** cpp)
{
    skiptok(cpp);
    
    return atoi(*cpp);
}

static void load_data(
    const char * dataset, 
    vector<int *> & scans,
    vector<long *> & odometries)
{
    char filename[256];
    
    sprintf(filename, "%s.dat", dataset);
    printf("Loading data from %s ... \n", filename);
    
    FILE * fp = fopen(filename, "rt");
    
    if (!fp)
    {
        fprintf(stderr, "Failed to open file\n");
        exit(1);
    }
    
    char s[MAXLINE];
    
    while (fgets(s, MAXLINE, fp))
    {
        char * cp = strtok(s, " ");
               
        long * odometry = new long [3];
        odometry[0] = atol(cp);
        skiptok(&cp);        
        odometry[1] = nextint(&cp);
        odometry[2] = nextint(&cp);
        
        odometries.push_back(odometry);
        
        // Skip unused fields
        for (int k=0; k<20; ++k)
        {
            skiptok(&cp);
        }
        
        int * scanvals = new int [SCAN_SIZE];
        
        for (int k=0; k<SCAN_SIZE; ++k)
        {
            scanvals[k] = nextint(&cp);
        }
        
        scans.push_back(scanvals);
    }
    
    fclose(fp);    
}

// Class for Mines verison of URG-04LX Lidar -----------------------------------

class MinesURG04LX : public URG04LX
{
    
public:
    
    MinesURG04LX(void): URG04LX(
        70,          // detectionMargin
        145)         // offsetMillimeters
    {
    }
};

// Class for MinesRover custom robot -------------------------------------------

class Rover : WheeledRobot
{
    
public:
    
    Rover() : WheeledRobot(
         77,     // wheelRadiusMillimeters
        165)     // halfAxleLengthMillimeters
    {
    }
    
    PoseChange computePoseChange(
            double timestamp,
            double left_wheel_odometry,
            double right_wheel_odometry)
    {  
        return WheeledRobot::computePoseChange(
                timestamp,
                left_wheel_odometry,
                right_wheel_odometry);
    }

protected:    
    
    void extractOdometry(
        double timestamp, 
        double leftWheelOdometry, 
        double rightWheelOdometry, 
        double & timestampSeconds, 
        double & leftWheelDegrees, 
        double & rightWheelDegrees)
    {        
        // Convert microseconds to seconds, ticks to angles        
        timestampSeconds = timestamp / 1e6;
        leftWheelDegrees = ticksToDegrees(leftWheelOdometry);
        rightWheelDegrees = ticksToDegrees(rightWheelOdometry);
    }
    
    void descriptorString(char * str)
    {
        sprintf(str, "ticks_per_cycle=%d", this->TICKS_PER_CYCLE);
    }
        
private:
    
    double ticksToDegrees(double ticks)
    {
        return ticks * (180. / this->TICKS_PER_CYCLE);
    }
    
    static const int TICKS_PER_CYCLE = 2000;
};

// Progress-bar class
// Adapted from http://code.activestate.com/recipes/168639-progress-bar-class/
// Downloaded 12 January 2014

class ProgressBar
{
public:
    
    ProgressBar(int minValue, int maxValue, int totalWidth)
    {
        strcpy(this->progBar, "[]");   // This holds the progress bar string
		this->min = minValue;
		this->max = maxValue;
		this->span = maxValue - minValue;
		this->width = totalWidth;
		this->amount = 0;       // When amount == max, we are 100% done 
		this->updateAmount(0);  // Build progress bar string
	}
	
    void updateAmount(int newAmount)
    {
		if (newAmount < this->min)
		{
		    newAmount = this->min;
		}
		if (newAmount > this->max)
		{
		    newAmount = this->max;
		}
		
		this->amount = newAmount;

		// Figure out the new percent done, round to an integer
		float diffFromMin = float(this->amount - this->min);
		int percentDone = (int)round((diffFromMin / float(this->span)) * 100.0);

		// Figure out how many hash bars the percentage should be
		int allFull = this->width - 2;
		int numHashes = (int)round((percentDone / 100.0) * allFull);

		
		// Build a progress bar with hashes and spaces
		strcpy(this->progBar, "[");
		this->addToProgBar("#", numHashes);
		this->addToProgBar(" ", allFull-numHashes);
		strcat(this->progBar, "]");
		
		// Figure out where to put the percentage, roughly centered
		int percentPlace =  (strlen(this->progBar) / 2) - ((int)(log10(percentDone+1)) + 1);
		char percentString[5];
		sprintf(percentString, "%d%%", percentDone);
				
		// Put it there
		for (int k=0; k<strlen(percentString); ++k)
		{
		    this->progBar[percentPlace+k] = percentString[k];
		}
		
    }
    
    char * str()
    {   
        return this->progBar;
    }
    
private:
    
    char progBar[1000]; // more than we should ever need
    int min;
    int max;
    int span;
    int width;
    int amount;
     
    void addToProgBar(const char * s, int n)
    {
        for (int k=0; k<n; ++k)
        {
            strcat(this->progBar, s);
        }
    }
};

// Helpers ----------------------------------------------------------------

int coords2index(double x,  double y)
{    
    return y * MAP_SIZE_PIXELS + x;
}


int mm2pix(double mm)
{
    return (int)(mm / (MAP_SIZE_METERS * 1000. / MAP_SIZE_PIXELS));  
}

int main( int argc, const char** argv )
{    
    // Bozo filter for input args
    if (argc < 3)
    {
        fprintf(stderr, 
            "Usage:   %s <dataset> <use_odometry> <random_seed>\n", 
            argv[0]);
        fprintf(stderr, "Example: %s exp2 1 9999\n", argv[0]);
        exit(1);
    }
    
    // Grab input args
    const char * dataset = argv[1];
    bool use_odometry    =  atoi(argv[2]) ? true : false;
    int random_seed =  argc > 3 ? atoi(argv[3]) : 0;
    
    // Load the Lidar and odometry data from the file   
    vector<int *> scans;
    vector<long *> odometries;
    load_data(dataset, scans, odometries);
       
    // Build a robot model in case we want odometry
    Rover robot = Rover();
    
    // Create a byte array to receive the computed maps
    unsigned char * mapbytes = new unsigned char[MAP_SIZE_PIXELS * MAP_SIZE_PIXELS];
        
    // Create SLAM object
    MinesURG04LX laser;
    SinglePositionSLAM * slam = random_seed ?
    (SinglePositionSLAM*)new RMHC_SLAM(laser, MAP_SIZE_PIXELS, MAP_SIZE_METERS, random_seed) :
    (SinglePositionSLAM*)new Deterministic_SLAM(laser, MAP_SIZE_PIXELS, MAP_SIZE_METERS);
	    
    // Report what we're doing
    int nscans = scans.size();
    printf("Processing %d scans with%s odometry / with%s particle filter...\n",
        nscans, use_odometry ? "" : "out", random_seed ? "" : "out");
    ProgressBar * progbar = new ProgressBar(0, nscans, 80); 
        
    // Start with an empty trajectory of positions
    vector<double *> trajectory;
    
    // Start timing
    time_t start_sec = time(NULL);

    // Loop over scans
    for (int scanno=0; scanno<nscans; ++scanno)
    {                         
        int * lidar = scans[scanno];
        
        // Update with/out odometry
        if (use_odometry)
        {
            long * o = odometries[scanno];
            PoseChange poseChange = robot.computePoseChange(o[0], o[1], o[2]);
            slam->update(lidar, poseChange);            
        }
        else
        {
            slam->update(lidar);  
        }
        
        Position position = slam->getpos();

        // Add new coordinates to trajectory
        double * v = new double[2];
        v[0] = position.x_mm;
        v[1] = position.y_mm;
        trajectory.push_back(v);     
        
        // Tame impatience
        progbar->updateAmount(scanno);
        printf("\r%s", progbar->str());
        fflush(stdout);
    }

    // Report speed
    time_t elapsed_sec = time(NULL) - start_sec;
    printf("\n%d scans in %ld seconds = %f scans / sec\n", 
           nscans, elapsed_sec, (float)nscans/elapsed_sec);
              
    // Get final map
    slam->getmap(mapbytes);

    // Put trajectory into map as black pixels
    for (int k=0; k<(int)trajectory.size(); ++k)
    {        
        double * v = trajectory[k];
                        
        int x = mm2pix(v[0]);
        int y = mm2pix(v[1]);
        
        delete v;
                        
        mapbytes[coords2index(x, y)] = 0;
    }
            
    // Save map and trajectory as PGM file    
    
    char filename[100];
    sprintf(filename, "%s.pgm", dataset);
    printf("\nSaving map to file %s\n", filename);
    
    FILE * output = fopen(filename, "wt");
    
    fprintf(output, "P2\n%d %d 255\n", MAP_SIZE_PIXELS, MAP_SIZE_PIXELS);
    
    for (int y=0; y<MAP_SIZE_PIXELS; y++)
    {
        for (int x=0; x<MAP_SIZE_PIXELS; x++)
        {
            fprintf(output, "%d ", mapbytes[coords2index(x, y)]);
        }
        fprintf(output, "\n");
    }
    
    printf("\n");
    
    // Clean up
    for (int scanno=0; scanno<(int)scans.size(); ++scanno)
    {                                       
        delete scans[scanno];
        delete odometries[scanno];
    }
    
    if (random_seed)
    {
        delete ((RMHC_SLAM *)slam);
    }
    else
    {
        delete ((Deterministic_SLAM *)slam);
    }

    delete progbar;
    delete mapbytes;
    fclose(output);

    
    return 0;
}
