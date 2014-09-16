%
% logdemo.m : BreezySLAM Matlab demo.  Reads logfile with odometry and scan 
%             data from Paris Mines Tech and displays the map and robot
%             trajectory.
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

% Map size, scale
MAP_SIZE_PIXELS          = 800;
MAP_SIZE_METERS          =  32;

% Grab input args

if nargin < 2
    use_odometry = 0;
end

if nargin < 3
    seed = 0;
end

% Load data from file
[scans, odometries] = load_data(dataset);

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

% Report what we're doing
nscans = size(scans, 1);
h = waitbar(0, sprintf('Processing %d scans %s odometry, %s filter', ...
    nscans, flag2with(use_odometry), flag2with(seed)));

% Start with an empty trajectory of positions
trajectory = zeros(nscans, 2);

% Start timing
start_sec = cputime;

% Loop over scans
for scanno = 1:nscans
    
    waitbar(scanno/nscans, h)
    
    if use_odometry
        
        % Convert odometry to velocities
        [velocities,robot] = robot.computeVelocities(odometries(scanno, :));
                
        % Update SLAM with lidar and velocities
        slam = slam.update(scans(scanno,:), velocities);
          
    else
        
        % Update SLAM with lidar alone
        slam = slam.update(scans(scanno,:));
    end
    
    % Get new position
    [x_mm, y_mm, ~] = slam.getpos();
    
    % Add new position to trajectory
    trajectory(scanno, :) = [x_mm, y_mm];
    
end

% Hide waitbar
close(h)

% Report elapsed time
elapsed_sec = cputime - start_sec;
fprintf('%d scans in %f sec = %f scans / sec\n', nscans, elapsed_sec, nscans/elapsed_sec)

                 
% Get final map
map = slam.getmap();

% Put trajectory into map as black pixels
for scanno = 1:nscans
    
    x_pix = mm2pix(trajectory(scanno,1), MAP_SIZE_METERS, MAP_SIZE_PIXELS);
    y_pix = mm2pix(trajectory(scanno,2), MAP_SIZE_METERS, MAP_SIZE_PIXELS);
    
    map(y_pix, x_pix) = 0;
    
end

% Display the final map and trajectory
figure
image(map/4) % Keep bytes in [0,64] for colormap
axis('square')
colormap('gray')


% Function to turn 0/1 to 'without'/'with' --------------------------------
function s = flag2with(x)
s = 'without';
if x
    s = 'with';
end

% Function to cnovert millimeters to pixels -------------------------------

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

function [scans,odometries] = load_data(dataset)

data = load([dataset '.dat']);

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
