classdef RMHC_SLAM < SinglePositionSLAM
    %RMHC_SLAM Random Mutation Hill-Climbing SLAM 
    %    Implements the getNewPosition() method of SinglePositionSLAM 
    %    using Random-Mutation Hill-Climbing search.  Uses its own internal 
    %    pseudorandom-number generator for efficiency.
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
   
    properties (Access = 'public')
        sigma_xy_mm = 100; % std. dev. of X/Y component of search
        sigma_theta_degrees = 20; % std. dev. of angular component of search
        max_search_iter = 1000; % max. # of search iterations per update
    end
    
    properties (Access = 'private')
        c_randomizer
    end
    
    methods
        
        function slam = RMHC_SLAM(laser, map_size_pixels, map_size_meters, random_seed)
        %Creates an RMHC_SLAM object suitable for updating with new Lidar and odometry data.
        %    slam = RMHC_SLAM(laser, map_size_pixels, map_size_meters, random_seed)
        %    laser is a Laser object representing the specifications of your Lidar unit
        %    map_size_pixels is the size of the square map in pixels
        %    map_size_meters is the size of the square map in meters
        %    random_seed supports reproducible results; defaults to system time if unspecified
            
            slam = slam@SinglePositionSLAM(laser, map_size_pixels, map_size_meters);
            
            if nargin < 3
                random_seed = floor(cputime) & hex2dec('FFFF');
            end
            
            slam.c_randomizer = mex_breezyslam('Randomizer_init', random_seed);
            
        end
        
        function new_pos = getNewPosition(slam, start_pos)
            % Implements the _getNewPosition() method of SinglePositionSLAM. 
            %    Uses Random-Mutation Hill-Climbing search to look for a 
            %    better position based on a starting position.
                                               
            [new_pos.x_mm,new_pos.y_mm,new_pos.theta_degrees] = ...
            mex_breezyslam('rmhcPositionSearch',  ...
                start_pos, ...
                slam.map.c_map, ...
                slam.scan_for_distance.c_scan, ...
                slam.laser,...
                slam.sigma_xy_mm,...
                slam.sigma_theta_degrees,...
                slam.max_search_iter,...
                slam.c_randomizer);
        end
        
    end
    
end

