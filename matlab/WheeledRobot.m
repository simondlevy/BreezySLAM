classdef WheeledRobot
    %WheeledRobot An abstract class supporting ododmetry for wheeled robots.
    %    Your implementing class should provide the method:
    %
    %       extractOdometry(obj, timestamp, leftWheel, rightWheel) -->
    %          (timestampSeconds, leftWheelDegrees, rightWheelDegrees)
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
        
        wheelRadiusMillimeters
        halfAxleLengthMillimeters
        
        timestampSecondsPrev
        leftWheelDegreesPrev
        rightWheelDegreesPrev
        
    end
    
    methods (Access = 'protected')
        
        function s = str(obj)
            % Returns a string representation of this WheeledRobot
            s = sprintf('<Wheel radius=%f mm Half axle Length=%f mm>', ...
                obj.wheelRadiusMillimeters, obj.halfAxleLengthMillimeters);
        end
        
        function robot = WheeledRobot(wheelRadiusMillimeters, halfAxleLengthMillimeters)
            % Constructs a WheeledRobot object
            %    robot = WheeledRobot(wheelRadiusMillimeters, halfAxleLengthMillimeters)
            robot.wheelRadiusMillimeters = wheelRadiusMillimeters;
            robot.halfAxleLengthMillimeters = halfAxleLengthMillimeters;
            
        end
        
        function r = deg2rad(~, d)
            % Converts degrees to radians
            r = d * pi / 180;
        end
        
    end
    
    methods (Abstract)
        
        extractOdometry(obj, timestamp, leftWheel, rightWheel)
        
    end
    
    methods (Access = 'public')
        
        function [poseChange, obj] = computePoseChange(obj, timestamp, leftWheelOdometry, rightWheelOdometry)
            % Computes forward and angular poseChange based on odometry.
            %
            % Parameters:
            
            % timestamp          time stamp, in whatever units your robot uses
            % leftWheelOdometry  odometry for left wheel, in whatever form your robot uses
            % rightWheelOdometry odometry for right wheel, in whatever form your robot uses
            %
            % Returns a tuple (dxyMillimeters, dthetaDegrees, dtSeconds)
            
            % dxyMillimeters     forward distance traveled, in millimeters
            % dthetaDegrees change in angular position, in degrees
            % dtSeconds     elapsed time since previous odometry, in seconds
            
            dxyMillimeters = 0;
            dthetaDegrees = 0;
            dtSeconds = 0;
            
            [timestampSecondsCurr, leftWheelDegreesCurr, rightWheelDegreesCurr] = ...
                obj.extractOdometry(timestamp, leftWheelOdometry, rightWheelOdometry);
            
            if obj.timestampSecondsPrev
                
                leftDiffDegrees = leftWheelDegreesCurr - obj.leftWheelDegreesPrev;
                rightDiffDegrees = rightWheelDegreesCurr - obj.rightWheelDegreesPrev;
                
                dxyMillimeters =  obj.wheelRadiusMillimeters * ...
                    (obj.deg2rad(leftDiffDegrees) + obj.deg2rad(rightDiffDegrees));
                
                dthetaDegrees =  (obj.wheelRadiusMillimeters / obj.halfAxleLengthMillimeters) * ...
                    (rightDiffDegrees - leftDiffDegrees);
                
                dtSeconds = timestampSecondsCurr - obj.timestampSecondsPrev;
            end
            
            % Store current odometry for next time
            obj.timestampSecondsPrev = timestampSecondsCurr;
            obj.leftWheelDegreesPrev = leftWheelDegreesCurr;
            obj.rightWheelDegreesPrev = rightWheelDegreesCurr;
            
            % Return linear velocity, angular velocity, time difference
            poseChange = [dxyMillimeters, dthetaDegrees, dtSeconds];
            
        end
    end    
end

