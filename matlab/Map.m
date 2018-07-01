classdef Map
    %A class for maps (occupancy grids) used in SLAM
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
    
    properties (Access = {?RMHC_SLAM})
        
        c_map
    end
    
    methods (Access = 'public')
        
        function map = Map(size_pixels, size_meters)
            % Map creates an empty square map
            %     map = Map(size_pixels, size_meters)
            map.c_map = mex_breezyslam('Map_init', size_pixels, size_meters);
            
        end
        
        function disp(map)
            % Displays data about this map
            mex_breezyslam('Map_disp', map.c_map)
            
        end
        
        function update(map, scan, new_position, map_quality, hole_width_mm)
            % Updates this map with a new scan and position
            %
            %     update(map, scan, new_position, map_quality, hole_width_mm)
            mex_breezyslam('Map_update', map.c_map, scan.c_scan, new_position, int32(map_quality), hole_width_mm)
            
        end
        
        function bytes = get(map)
            % Returns occupancy grid matrix of bytes for this map
            %     
            %     bytes = get(map)
            
            % Transpose for uniformity with Python, C++ versions
            bytes = mex_breezyslam('Map_get', map.c_map)';
            
        end
        
    end % methods
    
end % classdef
