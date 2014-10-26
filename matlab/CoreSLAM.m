classdef CoreSLAM
    %CoreSLAM CoreSLAM abstract class for BreezySLAM
    %     CoreSLAM is an abstract class that uses the classes Position, 
    %     Map, Scan, and Laser to run variants of the simple CoreSLAM 
    %     (tinySLAM) algorithm described in 
    %         
    %      @inproceedings{coreslam-2010,  
    %        author    = {Bruno Steux and Oussama El Hamzaoui}, 
    %        title     = {CoreSLAM: a SLAM Algorithm in less than 200 lines of C code},  
    %        booktitle = {11th International Conference on Control, Automation,   
    %                     Robotics and Vision, ICARCV 2010, Singapore, 7-10  
    %                     December 2010, Proceedings},  
    %        pages     = {1975-1979},  
    %        publisher = {IEEE},  
    %        year      = {2010}
    %      }
    %     
    %     
    %    Implementing classes should provide the method
    %     
    %       updateMapAndPointcloud(scan_mm, velocities)
    %     
    %    to update the map and point-cloud (particle cloud).
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
        map_quality = 50;     % The quality of the map (0 through 255); default = 50
        hole_width_mm = 600;  % The width in millimeters of each "hole" in the map (essentially, wall width); default = 600
    end
    
    properties (Access = 'protected')
        laser % (internal)
        scan_for_distance % (internal)
        scan_for_mapbuild % (internal)
        map % (internal)
        velocities % (internal)
    end
       
    methods (Abstract, Access = 'protected')
        
        updateMapAndPointcloud(slam, velocities)
        
    end
    
    methods (Access = 'private')
        
        function scan_update(slam, scanobj, scans_mm)
            scanobj.update(scans_mm, slam.hole_width_mm, slam.velocities)
        end
        
    end    
    
    methods (Access = 'protected')
        
        function slam = CoreSLAM(laser, map_size_pixels, map_size_meters)
            % Creates a CoreSLAM object suitable for updating with new Lidar and odometry data.
            % CoreSLAM(laser, map_size_pixels, map_size_meters)
            %   laser is a Laser object representing the specifications of your Lidar unit
            %   map_size_pixels is the size of the square map in pixels
            %   map_size_meters is the size of the square map in meters
           
            % Store laser for later
            slam.laser = laser;
            
            % Initialize velocities (dxyMillimeters, dthetaDegrees, dtSeconds) for odometry
            slam.velocities = [0, 0, 0];
            
            % Initialize a scan for computing distance to map, and one for updating map
            slam.scan_for_distance = Scan(laser, 1);
            slam.scan_for_mapbuild = Scan(laser, 3);
            
            % Initialize the map
            slam.map = Map(map_size_pixels, map_size_meters);
            
        end
    end
    
    methods (Access = 'public')
        
        function slam = update(slam, scans_mm, velocities)
            % Updates the scan and odometry.
            %     Calls the the implementing class's updateMapAndPointcloud()
            %     method with the specified velocities.
            %
            %     slam = update(slam, scans_mm, [velocities])
            %
            %     scan_mm is a list of Lidar scan values, whose count is specified in the scan_size
            %     velocities is an optional list of velocities [dxy_mm, dtheta_degrees, dt_seconds] for odometry
                                    
            % Build a scan for computing distance to map, and one for updating map
            slam.scan_update(slam.scan_for_mapbuild, scans_mm)            
            slam.scan_update(slam.scan_for_distance, scans_mm)
                                        
            % Default to zero velocities
            if nargin < 3
                velocities = [0, 0, 0];
            end
            
            % Update velocities
            velocity_factor = 0;
            if velocities(3) > 0
                velocity_factor = 1 / velocities(3);
            end
            new_dxy_mm = velocities(1) * velocity_factor;
            new_dtheta_degrees = velocities(2) * velocity_factor;
            slam.velocities = [new_dxy_mm, new_dtheta_degrees, 0];
            
            % Implementing class updates map and pointcloud
            slam = slam.updateMapAndPointcloud(velocities);
            
        end
        
        function map = getmap(slam)
            % Returns the current map.
            %     Map is returned as an occupancy grid (matrix of pixels).
            map = slam.map.get();
        end
        
    end
    
end

