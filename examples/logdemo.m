%
% logdemo.m : BreezySLAM Matlab demo.  Reads logfile with odometry and scan
%             data from Paris Mines Tech and displays the map and robot
%             position in real time.
%
% Usage:
%
%   >> logdemo(dataset, [use_odometry], [random_seed])
%
% Examples:
%
%   >> logdemo('exp2')
%
% For details see
%
%     @inproceedings{coreslam-2010,
%       author    = {Bruno Steux and Oussama El Hamzaoui},
%       title     = {CoreSLAM: a SLAM Algorithm in less than 200 lines of C code},
%       booktitle = {11th International Conference on Control, Automation,
%                    Robotics and Vision, ICARCV 2010, Singapore, 7-10
%                    December 2010, Proceedings},
%       pages     = {1975-1979},
%       publisher = {IEEE},
%       year      = {2010}
%     }
%
% Copyright (C) 2014 Simon D. Levy
%
% This code is free software: you can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License as
% published by the Free Software Foundation, either version 3 of the
% License, or (at your option) any later version.
%
% This code is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU General Public License for more details.
%
% You should have received a copy of the GNU Lesser General Public License
% along with this code.  If not, see <http://www.gnu.org/licenses/>.

function logdemo(dataset, use_odometry, seed)

% Params

MAP_SIZE_PIXELS          = 800;
MAP_SIZE_METERS          = 32;
ROBOT_SIZE_PIXELS        = 10;

% Grab input args

if nargin < 2
    use_odometry = 0;
end

if nargin < 3
    seed = 0;
end

% Load data from file
[times, scans, odometries] = load_data(dataset);

% Build a robot model if we want odometry
robot = [];
if use_odometry
    robot = MinesRover();
end

% Create a CoreSLAM object with laser params and optional robot object
if seed
    slam = RMHC_SLAM(MinesLaser(), MAP_SIZE_PIXELS, MAP_SIZE_METERS, seed);
else
    slam = Deterministic_SLAM(MinesLaser(), MAP_SIZE_PIXELS, MAP_SIZE_METERS);
end

% Initialize previous time for delay
prevTime = 0;

% Loop over scans
for scanno = 1:size(scans, 1)
    
    if use_odometry
        
        % Convert odometry to velocities
        [velocities,robot] = robot.computePoseChange(odometries(scanno, :));
        
        % Update SLAM with lidar and velocities
        slam = slam.update(scans(scanno,:), velocities);
        
    else
        
        % Update SLAM with lidar alone
        slam = slam.update(scans(scanno,:));
    end
        
    % Get new position
    [x_mm, y_mm, theta_degrees] = slam.getpos();
    
    % Get current map
    map = slam.getmap();
        
    % Display map
    hold off
    image(map/4) % Keep bytes in [0,64] for colormap
    axis('square')
    colormap('gray')
    hold on
    
    % Generate a polyline to represent the robot
    [x_pix, y_pix] = robot_polyline(ROBOT_SIZE_PIXELS);

    % Rotate the polyline based on the robot's angular rotation
    theta_radians = pi * theta_degrees / 180;
    x_pix_r = x_pix * cos(theta_radians) - y_pix * sin(theta_radians);
    y_pix_r = x_pix * sin(theta_radians) + y_pix * cos(theta_radians);

    % Add the robot's position as offset to the polyline
    x_pix = x_pix_r + mm2pix(x_mm, MAP_SIZE_METERS, MAP_SIZE_PIXELS);
    y_pix = y_pix_r + mm2pix(y_mm, MAP_SIZE_METERS, MAP_SIZE_PIXELS);
    
    % Add robot image to map
    fill(x_pix, y_pix, 'r')
    drawnow   
    
    % Delay based on current time in microseconds
    currTime = times(scanno);
    if scanno > 1
        pause((currTime-prevTime)/1e6)
    end
    prevTime = times(scanno);
    
    
end

% Function to generate a x, y points for a polyline to display the robot
% Currently we just generate a triangle.
function [x_pix, y_pix] = robot_polyline(robot_size)
HEIGHT_RATIO = 1.25; 
x_pix = [-robot_size,              robot_size, -robot_size]; 
y_pix = [-robot_size/HEIGHT_RATIO, 0 ,          robot_size/HEIGHT_RATIO];

% Function to convert millimeters to pixels -------------------------------

function pix = mm2pix(mm, MAP_SIZE_METERS, MAP_SIZE_PIXELS)
pix = floor(mm / (MAP_SIZE_METERS * 1000. / MAP_SIZE_PIXELS));


% Function to load all from file ------------------------------------------
% Each line in the file has the format:
%
%  TIMESTAMP  ... Q1  Q1 ... Distances
%  (usec)                    (mm)
%  0          ... 2   3  ... 24 ...
%
%where Q1, Q2 are odometry values

function [times, scans,odometries] = load_data(dataset)

data = load([dataset '.dat']);

times = data(:,1);
scans = data(:,25:end-1); % avoid final ' '
odometries = data(:,[1,3,4]);

% Function to build a Laser data structure for the Hokuyo URG04LX used for
% collecting the logfile data.

function laser = MinesLaser()

laser.scan_size = 682;
laser.scan_rate_hz = 10;
laser.detection_angle_degrees = 240;
laser.distance_no_detection_mm = 4000;
laser.detection_margin = 70;
laser.offset_mm = 145;
