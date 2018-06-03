classdef MinesRover < WheeledRobot
    %MinesRover Class for MinesRover custom robot
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
        TICKS_PER_CYCLE = 2000;
    end
    
    methods (Access = 'private')
        
        function degrees = ticks_to_degrees(obj, ticks)
            
            degrees = ticks * (180. / obj.TICKS_PER_CYCLE);
            
        end
        
    end
    
    methods  (Access = 'public')
        
        function robot = MinesRover()
            
            robot = robot@WheeledRobot(77, 165);
            
        end
        
        function disp(obj)
            % Displays information about this MinesRover
            
            fprintf('<%s ticks_per_cycle=%d>\n', obj.str(), obj.TICKS_PER_CYCLE)
            
        end
        
        function [poseChange, obj] = computePoseChange(obj, odometry)
            
            [poseChange, obj] = computePoseChange@WheeledRobot(obj, odometry(1), odometry(2), odometry(3));
            
        end
        
        function [timestampSeconds, leftWheelDegrees, rightWheelDegrees] = ...
                extractOdometry(obj, timestamp, leftWheel, rightWheel)
            
            % Convert microseconds to seconds
            timestampSeconds = timestamp / 1e6;
            
            % Convert ticks to angles
            leftWheelDegrees = obj.ticks_to_degrees(leftWheel);
            rightWheelDegrees = obj.ticks_to_degrees(rightWheel);
            
        end
        
    end
    
end

