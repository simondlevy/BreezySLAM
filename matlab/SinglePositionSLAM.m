classdef SinglePositionSLAM < CoreSLAM
    %SinglePositionSLAM Abstract class for CoreSLAM with single-point cloud
    %    Implementing classes should provide the method
    %
    %    getNewPosition(self, start_position)
    %   
    %    to compute a new position based on searching from a starting position. 
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
    
    properties (Access = 'private')
        position
    end
    
    methods (Abstract)
        getNewPosition(slam, start_pos)
    end
    
    methods (Access = 'private')
        
        function c = costheta(slam)
            c = cosd(slam.position.theta_degrees);
        end
        
        function s = sintheta(slam)
            s = sind(slam.position.theta_degrees);
        end        
               
    end
    
    methods (Access = 'public')
        
        function slam = SinglePositionSLAM(laser, map_size_pixels, map_size_meters)
            
            slam = slam@CoreSLAM(laser, map_size_pixels, map_size_meters);
            
            % Initialize the position (x, y, theta)
            init_coord_mm = 500 * map_size_meters; % center of map
            slam.position.x_mm =  init_coord_mm;
            slam.position.y_mm =  init_coord_mm;
            slam.position.theta_degrees =  0;
            
        end
        
        function [x_mm, y_mm, theta_degrees] = getpos(slam)
            % Returns the current position.
            %     [x_mm, y_mm, theta_degrees] = getpos(slam)
            
            x_mm = slam.position.x_mm;
            y_mm = slam.position.y_mm;
            theta_degrees = slam.position.theta_degrees;
        end
        
    end
        
    methods (Access = 'protected')

        function slam = updateMapAndPointcloud(slam, velocities)
            
            % Start at current position
            start_pos = slam.position;
            
            % Add effect of velocities
            start_pos.x_mm = start_pos.x_mm + velocities(1) * slam.costheta();
            start_pos.y_mm = start_pos.y_mm + velocities(1) * slam.sintheta();
            start_pos.theta_degrees = start_pos.theta_degrees + velocities(2);
            
            % Add offset from laser
            start_pos.x_mm  = start_pos.x_mm + slam.laser.offset_mm * slam.costheta();
            start_pos.y_mm  = start_pos.y_mm + slam.laser.offset_mm * slam.sintheta();
            
            % Get new position from implementing class
            new_position = slam.getNewPosition(start_pos);
                                                
            % Update the map with this new position
            slam.map.update(slam.scan_for_mapbuild, new_position, slam.map_quality, slam.hole_width_mm)
                        
            % Update the current position with this new position, adjusted by laser offset
            slam.position = new_position;
            slam.position.x_mm = slam.position.x_mm - slam.laser.offset_mm * slam.costheta();
            slam.position.y_mm = slam.position.y_mm - slam.laser.offset_mm * slam.sintheta();
            
        end
        
    end
    
end

