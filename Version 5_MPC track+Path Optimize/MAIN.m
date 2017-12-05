% This GOAL of this project is to ultilize the map and laser scan data to find the obstacle-free optimal path
% and use MPC to tracking the optimal path with obstacle-avoidance
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Using RobotSimulator
clear;
close all;
clc;

% Set robotsimulator and enable function
robotRadius = 0.3;
%%
robot = RobotSimulator();
%%
robot.enableLaser(true);
robot.setRobotSize(robotRadius);
robot.showTrajectory(true);
% 
%% Set start point and goal
% Set START to BEGIN
%%
%Test 1
%startLocation = [2.725 14.08];
%initialOrientation = -pi/4;
%%
%Test 2
startLocation = [2.275 0.775];
initialOrientation = pi/4;
%%
figure(1)
hold all
plot(startLocation(1),startLocation(2),'o')

% Set GOAL to REACH
endLocation = [14.38 2.225];
figure(1)
hold all
plot(endLocation(1),endLocation(2),'x')

%% Set up the inial position and pose

robotCurrentLocation = startLocation;
robotCurrentPose = [robotCurrentLocation initialOrientation];
robot.setRobotPose(robotCurrentPose);

plan_path=[];

%% Use MPC to do path planing
% Continue use MPC and PRM until reach the goal or hit the obstacle
% Get optimized PRM circle waypoints as z_ref
mapInflated = copy(robot.Map);
inflate(mapInflated,robotRadius);
%% Here is where the map gets inflated
optPRMPoints=getOptimalPRMPoints1(mapInflated,startLocation,endLocation)
PointNo=2
%%
while norm(robotCurrentPose(1:2) - endLocation)>0.1
    yalmip('clear')
    if norm(robotCurrentPose(1:2)-optPRMPoints(PointNo,:))<0.8
        PointNo=PointNo+1
    end
    if PointNo==size(optPRMPoints,1)
        z_ref=endLocation;
    else
        z_ref = optPRMPoints(PointNo,:)
    end
    
    pose = robot.getRobotPose;
    [range,angle] = robot.getRangeData;
    laser=[range angle];
%     bb = zeros(21,1);
%     x = aa(1) + bb;
%     y = aa(2) + bb;
%     beta = aa(3) + bb;
    
%    data=[(180/pi)*theta range (180/pi)*angle (180/pi).*(angle+theta) x+range.*cos(angle+theta) y+range.*sin(angle+theta)];
    % Write laser data and use it in MPC

%     o_data = [];
%     for i = 1:21
%       if ~isnan(data(i,2)) 
%           o_data = [o_data; data(i,5:6)]; %[range angle]
%       end
%     end
%     obs_ref = [];
%     % Filter: exlude obstacle data that's 20m farther from robot 
%     for i = 1:size(o_data, 1)
%         if o_data(i,1)<20
%             obs_ref = [obs_ref;odata(i,:)]; % [range angle]
%         end
%     end

    % Using MPC path planer
    robotCurrentPose = robot.getRobotPose;
    [get_path,sol] = mpc_controller(robotCurrentPose,z_ref,laser);
    
    % If the result is not successfully solved, then use the previous ones.
    % And reduce the size if use the previous ones.
    % If the path size too small to use, then use PRM get new plan_path.
    if sol.problem == 0
        plan_path = get_path;
%     elseif size(plan_path,1) >= 8
%         plan_path = plan_path(8:end,:);
    else
        % if MPC doesn't solve, then drive to the next optPRM point
        plan_path = optPRMPoints(PointNo:end,:);
        if norm(robotCurrentPose(1:2)-optPRMPoints(PointNo,:))<0.8
            plan_path = optPRMPoints(PointNo+1:end,:);
        end
    end
    
        
%         Copy the curent path and inflate each occupancy grid
%         mapInflated = copy(robot.Map);
%         inflate(mapInflated,robotRadius);
%         
%         % Using PRM (probolistic roadmap method) to find path
%         prm = robotics.PRM(mapInflated);
%         % Set # of random points
%         prm.NumNodes = 200;
%         prm.ConnectionDistance = 1;
%         
%         plan_path = findpath(prm, robotCurrentPose(1:2), endLocation);
        
%         while isempty(plan_path)
%             % No feasible path found yet, increase the number of nodes
%             prm.NumNodes = prm.NumNodes + 50;
%             
%             % Use the |update| function to re-create the PRM roadmap with the changed
%             % attribute
%             update(prm);
%             
%             % Search for a feasible path with the updated PRM
%             plan_path = findpath(prm,robotCurrentPose(1:2), endLocation);
%         end
        
%     end
    
    figure(1)
    hold all
    
    plot(plan_path(:,1),plan_path(:,2),'x')
    
    %  Use Pure Pursuit to contorl the car
    controller = robotics.PurePursuit;
    
    % Feed the middle point of plan_path to the pursuit controller
%     if size(plan_path,1) >= 10
%         controller.Waypoints = plan_path(1:10,:);
%     else
    controller.Waypoints = plan_path(1:ceil(end/2),:);
%     end
    
    % The maximum angular velocity acts as a saturation limit for rotational velocity
    controller.DesiredLinearVelocity = 0.4;
    controller.MaxAngularVelocity = 8;
    
    % As a general rule, the lookahead distance should be larger than the desired
    % linear velocity for a smooth path. The robot might cut corners when the
    % lookahead distance is large. In contrast, a small lookahead distance can
    % result in an unstable path following behavior. A value of 0.6 m was chosen
    % for this example.
    controller.LookaheadDistance = 0.5;
    
    % The controller runs at 10 Hz.
    controlRate = robotics.Rate(10);
    
    % Set conditon for loop leaving
    robotCurrentLocation = robotCurrentPose(1:2);
    robotGoal = controller.Waypoints(end,:);
    distanceToGoal = norm(robotCurrentLocation - robotGoal);
    
    % Drive robot 50 times or close(0.02) to desired path end point
    flag=0;
    while ( distanceToGoal > 0.05 && flag < 50)
        [v, omega] = controller(robot.getRobotPose);
        drive(robot, v, omega);
        robotCurrentPose = robot.getRobotPose;
        distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal);
        flag = flag + 1;
        waitfor(controlRate);
    end
    
    %drive(robot, v, omega);
end

