%%
clear;
close all;
clc;
robotRadius = 0.15;
robot = RobotSimulator();
robot.enableLaser(true);
robot.setRobotSize(robotRadius);
robot.showTrajectory(true);


%% First time path planning
mapInflated = copy(robot.Map);
inflate(mapInflated,robotRadius);
prm = robotics.PRM(mapInflated);
prm.NumNodes = 200;
prm.ConnectionDistance = 10;

% set start point and goal
startLocation = [4 12.5];
endLocation = [28 13];
path = findpath(prm, startLocation, endLocation);

while isempty(path)
    % No feasible path found yet, increase the number of nodes
    prm.NumNodes = prm.NumNodes + 10;

    % Use the |update| function to re-create the PRM roadmap with the changed
    % attribute
    update(prm);

    % Search for a feasible path with the updated PRM
    path = findpath(prm, startLocation, endLocation);
end

show(prm, 'Map', 'off', 'Roadmap', 'off')


%% set up the inial position
initialOrientation = 0;
robotCurrentLocation = startLocation;
robotCurrentPose = [robotCurrentLocation initialOrientation];
robot.setRobotPose(robotCurrentPose);


%%
%% Use MPC to do path planing
z_ref = endLocation;
  aa=robot.getRobotPose;
  bb=zeros(21,1);
  x = aa(1)+bb;
  y = aa(2)+bb;
  theta=aa(3)+bb;
  [range,angle]=robot.getRangeData;
  data=[ (180/pi)*theta range (180/pi)*angle (180/pi).*(angle+theta) x+range.*cos(angle+theta) y+range.*sin(angle+theta)];
  o_data=[];
  for i=1:21
      if ~isnan(data(i,2)) 
          o_data=[o_data; data(i,5:6)];
      end
  end
  ob_data=o_data;
 % get distance and filter 
  o_data(:,1)=o_data(:,1)-aa(1);
  o_data(:,2)=o_data(:,2)-aa(2);
  
  get_dis=[];
  for i=1:size(o_data,1)
      get_dis=[get_dis;norm(o_data(i,:))];
  end
  
  obs_ref=[];
  for i=1:size(get_dis,1)
      if get_dis(i)<=4
         obs_ref=[obs_ref;ob_data(i,:)];
      end
  end
          
robotCurrentPose = robot.getRobotPose;
[plan_path] = mpc_controller(robotCurrentPose,z_ref,obs_ref);
figure(1)
hold all
plot(plan_path(:,1),plan_path(:,2),'o')
%%  Use Pure Pursuit to contorl the car
controller = robotics.PurePursuit
controller.Waypoints = plan_path(1:10,:);
controller.DesiredLinearVelocity = 0.8;
controller.MaxAngularVelocity = pi;
controller.LookaheadDistance = 0.5;
controlRate = robotics.Rate(10);
robotCurrentLocation = robotCurrentPose(1:2);
robotGoal = controller.Waypoints(end,:);
distanceToGoal = norm(robotCurrentLocation - robotGoal);
while ( distanceToGoal > 0.05)
    [v, omega] = controller(robot.getRobotPose);
    drive(robot, v, omega);
    robotCurrentPose = robot.getRobotPose;
    distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal);
    waitfor(controlRate);
    
end

%drive(robot, v, omega);
