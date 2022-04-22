%% Speed dependent collision avoidance %% 
% $
clear; clc; close all;
%% SETUP
% Connect to ROS master
rosshutdown;
robotIp = '192.168.0.100'; % Kinetic
rosinit(robotIp);

% Subscribing and receiving for sensor data (odometery and LiDAR)
handles.odomSub = rossubscriber('/odom','Buffersize',5);
receive(handles.odomSub,3);
handles.laserSub = rossubscriber('/scan','BufferSize',5);
receive(handles.laserSub,3);

% Parameters used for pose graph mapping (tune if required+)
mapResolotion = 50;
maxLidarRange = 4.5;
lidarSlam = robotics.LidarSLAM(mapResolotion, maxLidarRange);

lidarSlam.LoopClosureThreshold = 100;
lidarSlam.LoopClosureSearchRadius = 4;

% Publisher for controlling the robot
joy=vrjoystick(1);
myParam=Simulink.Parameter;  % Steering control
myParam1=Simulink.Parameter; % Throttle position (velocity control)
myParam2=Simulink.Parameter; % forward or reverse

freq = 10;
runtime = 120;

r = rateControl(freq);

reset(r);

% Initialize the counter
i=0;
stop_flag = 0;
obstacle_dis = 15;

% vector for velocity and threshold distance
t_delay = 1;
v_var = [0.05, 0.1, 0.15, 0.2, 0.25];
d_var = v_var.*t_delay;

%% Combined loop to save data, check for collision (speed dependent)
while r.TotalElapsedTime < runtime
    
    i=i+1;
    laserScan = receive(handles.laserSub);
    ranges = double(laserScan.Ranges);
    angles = double(laserScan.readScanAngles);
    anglesDeg = rad2deg(angles);
    
    scan = lidarScan(ranges,angles);
    scan_filtered = scanFilter(scan);
    odom_data = receive(handles.odomSub);
    v_f = odom_data.Twist.Twist.Linear.X; 
    
    if stop_flag
        min_dis = obstacle_dis;
    else
        min_dis = abs(interp1(v_var, d_var, v_f, 'linear', 'extrap'));
    end

    [stop_flag, obstacle_dis] = collisionCheck(scan_filtered, min_dis);
    
    myParam.Value=axis(joy,1);
    myParam2.Value=button(joy,23);
    
    if stop_flag && ~myParam2.Value 
        myParam1.Value = 1;
        disp('Imminent Collision: emergency STOP activated!');
    else
        myParam1.Value=axis(joy,3);
    end
    
    set_param(bdroot,'SimulationCommand','Update')
    waitfor(r);
end

