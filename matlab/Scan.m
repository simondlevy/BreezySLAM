classdef Scan 
    %A class for Lidar scans used in SLAM
    %
    %    Copyright (C) 2014 Simon D. Levy
    % 
    %    This code is free software: you can redistribute it and/or modify
    %    it under the terms of the GNU Lesser General Public License as 
    %    published by the Free Software Foundation, either version 3 of the 
    %    License, or (at your option) any later version.
    % 
    %    This code is distributed in the hope that it will be useful,     
    %    but WITHOUT ANY WARRANTY without even the implied warranty of
    %    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    %    GNU General Public License for more details.
    % 
    %    You should have received a copy of the GNU Lesser General Public License 
    %    along with this code.  If not, see <http:#www.gnu.org/licenses/>.
   
   properties (Access = {?Map, ?RMHC_SLAM})
       
       c_scan
   end
   
   methods
       
      function scan = Scan(laser, span)
          % Creates a new scan object
          %     scan  = Scan(laser, [span]) 
          %     laser is a structure with the fields:
          %         scan_size
          %         scan_rate_hz
          %         detection_angle_degrees
          %         distance_no_detection_mm 
          %         distance_no_detection_mm
          %         detection_margin
          %         offset_mm = offset_mm
          %     span (default=1) supports spanning laser scan to cover the space better

          if nargin < 2
              span = 1;
          end
                              
          scan.c_scan = mex_breezyslam('Scan_init', laser, span);
                              
      end 
      
      function disp(scan)
          % Displays information about this Scan
          
          mex_breezyslam('Scan_disp', scan.c_scan)
         
      end 
      
      function update(scan, scans_mm, hole_width_mm, velocities)
          % Updates scan with new lidar and velocity data
          %    update(scans_mm, hole_width_mm, [velocities])
          %    scans_mm is a list of integers representing scanned distances in mm.
          %    hole_width_mm is the width of holes (obstacles, walls) in millimeters.
          %    velocities is an optional list[dxy_mm, dtheta_degrees]
          %    i.e., robot's (forward, rotational velocity) for improving the quality of the scan.
          mex_breezyslam('Scan_update', scan.c_scan, int32(scans_mm), hole_width_mm, velocities)
      end
      
   end
   
end 
