%%
clear;
close all;
clc;
robotRadius = 0.15;
robot = RobotSimulator();
robot.enableLaser(true);
robot.setRobotSize(robotRadius);
robot.showTrajectory(true);


%%
mapInflated = copy(robot.Map);
inflate(mapInflated,robotRadius);
prm = robotics.PRM(mapInflated);
prm.NumNodes = 200;
prm.ConnectionDistance = 10;

startLocation = [4 12.5];
endLocation = [28 13];
path = findpath(prm, startLocation, endLocation)

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
%%
goalRadius = 0.15;
robotCurrentLocation = path(1,:);
robotGoal = path(end,:);
distanceToGoal = norm(robotCurrentLocation - robotGoal);
controlRate = robotics.Rate(1);


%%
initialOrientation = 0;

robotCurrentPose = [robotCurrentLocation initialOrientation];

z_ref = [28 13]
%%
robot.setRobotPose(robotCurrentPose);
%%
while( distanceToGoal > goalRadius )

    
    %% Compute the controller outputs, i.e., the inputs to the robot
    %z_ref = [path(10,1) path(10,2)]

    [v,omega,z_next]=mpc_controller(robotCurrentPose,z_ref);
    
    % Simulate the robot using the controller outputs
    
    drive(robot, v, omega);
    robotCurrentPose = robot.getRobotPose;
    
    % Extract current location information from the current pose
    

%%
  aa=robot.getRobotPose;
  bb=zeros(21,1);
  x = aa(1)+bb;
  y = aa(2)+bb;
  theta=(180/pi)*aa(3)+bb;
  [range,angle]=robot.getRangeData;
robotCurrentPose = robot.getRobotPose;
[v,omega]=mpc_controller(robotCurrentPose,z_ref)
drive(robot, v, omega);

 %%   
    % Re-compute the distance to the goal
    
    aa=robot.getRobotPose;
    bb=zeros(21,1);
    x = aa(1)+bb;
    y = aa(2)+bb;
    theta=(180/pi)*aa(3)+bb;
    distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal);
    [range,angle]=robot.getRangeData;
    data = [v, omega]
    data=[ theta range (180/pi)*angle (180/pi).*angle+theta x+range.*cos(angle+theta) y+range.*sin(angle+theta)]
    waitfor(controlRate);

end


%%
%%
z_ref = [28 13];
  aa=robot.getRobotPose;
  bb=zeros(21,1);
  x = aa(1)+bb;
  y = aa(2)+bb;
  theta=(180/pi)*aa(3)+bb;
  [range,angle]=robot.getRangeData;
  data=[ theta range (180/pi)*angle (180/pi).*angle+theta x+range.*cos(angle+theta) y+range.*sin(angle+theta)]
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
[new_path,plan_path] = mpc_controller(robotCurrentPose,z_ref,obs_ref)
figure(1)
hold all
for i=1:20
    plot(plan_path(i,1),plan_path(i,2),'*')
end
%%
controller = robotics.PurePursuit
controller.Waypoints = new_path(10,:);
controller.DesiredLinearVelocity = 0.8;
controller.MaxAngularVelocity = pi;
controller.LookaheadDistance = 0.5;
goalRadius = 0.1;
controlRate = robotics.Rate(10);
robotCurrentLocation = robotCurrentPose(1:2);
robotGoal = new_path(end,:);
distanceToGoal = norm(robotCurrentLocation - robotGoal);
while ( distanceToGoal > 0.05)
    [v, omega] = controller(robot.getRobotPose);
    drive(robot, v, omega);
    robotCurrentPose = robot.getRobotPose;
    distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal);
    waitfor(controlRate);
    
end

%drive(robot, v, omega);

