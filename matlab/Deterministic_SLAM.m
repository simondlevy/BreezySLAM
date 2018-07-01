classdef Deterministic_SLAM < SinglePositionSLAM
    %Deterministic_SLAM SLAM with no Monte Carlo search
    %    Implements the getNewPosition() method of SinglePositionSLAM 
    %    by copying the start position.
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
       
    methods
        
        function slam = Deterministic_SLAM(laser, map_size_pixels, map_size_meters)
        %Creates a Deterministic_SLAM object suitable for updating with new Lidar and odometry data.
        %    slam = Deterministic_SLAM(laser, map_size_pixels, map_size_meters)
        %    laser is a Laser object representing the specifications of your Lidar unit
        %    map_size_pixels is the size of the square map in pixels
        %    map_size_meters is the size of the square map in meters
            
            slam = slam@SinglePositionSLAM(laser, map_size_pixels, map_size_meters);
            
        end
        
        function new_pos = getNewPosition(~, start_pos)
            % Implements the _getNewPosition() method of SinglePositionSLAM. 
            %    new_pos = getNewPosition(~, start_pos) simply returns start_pos
            
            new_pos = start_pos;
        end
        
    end
    
end

