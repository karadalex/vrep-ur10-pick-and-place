%% Start connection to V-REP
% IMPORTANT: for each successful call to simxStart, there
% should be a corresponding call to simxFinish at the end!
disp('Program started');
% vrep=remApi('remoteApi','extApi.h'); % using the header (requires a compiler)
vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

%% Load target positions, orientations and paths
referenceFrames
addpath('./ur-fwd-inv-kinematics');

path = zeros(4,4,26);
path_q = zeros(26,6);

home = zeros(1,6);
home(2) = -pi/2;
home(4) = -pi/2;

path_q(1,:) = home;
path_q(26,:) = home;
    
% solve IK for start and target positions of bottles
k = 2;
for i=1:4
    % PICK
    % Above starting point
    path(:,:,k) = M_0_6_bis(:,:,i)*pose(0,0,-0.2,0,0,0);
    th = invKin(path(:,:,k), M_joints, L, d, a);
    path_q(k, :) = th(:,1);
    k = k + 1;
    
    % At starting point
    path(:,:,k) = M_0_6_bis(:,:,i);
    th = invKin(path(:,:,k), M_joints, L, d, a);
    path_q(k, :) = th(:,1);
    k = k + 1;
    
    % Above starting point (after having picked the bottle)
    path(:,:,k) = M_0_6_bis(:,:,i)*pose(0,0,-0.4,0,0,0);
    th = invKin(path(:,:,k), M_joints, L, d, a);
    path_q(k, :) = th(:,1);
    k = k + 1;
    
    % PLACE
    % Approaching target point
    path(:,:,k) = M_0_6_bit(:,:,i)*pose(0,0,-0.5,0,0,0);
    th = invKin(path(:,:,k), M_joints, L, d, a);
    path_q(k, :) = th(:,1);
    k = k + 1;
    
    % At target point
    path(:,:,k) = M_0_6_bit(:,:,i);
    th = invKin(path(:,:,k), M_joints, L, d, a);
    path_q(k, :) = th(:,1);
    k = k + 1;
    
    % Leaving target point (after having placed the bottle)
    path(:,:,k) = M_0_6_bit(:,:,i)*pose(0,0,-0.5,0,0,0);
    th = invKin(path(:,:,k), M_joints, L, d, a);
    path_q(k, :) = th(:,1);
    k = k + 1;
end


%% Position-velocities plots
figure('Name', 'Position-Velocity diagrams in joint space (non-smoothed path)');
path_dot = zeros(26,6);
for i=1:6
    for j=1:25
        path_dot(j,i) = (path_q(j+1,i)-path_q(j,i))/2;
    end
    subplot(3,2,i);
    plot(1:26, path_q(:,i), 'b', 1:26, path_dot(:,i), 'g');
    title("q"+i);
    xlabel("steps");
    ylabel("rads, rads/s");
end

figure('Name', 'Position-Velocity diagrams in task space (non-smoothed path)');
path_xyzrpy = zeros(26,6);
path_xyzrpy_dot = zeros(26,6);
plot_titles = ["x", "y", "z", "roll", "pitch", "yaw"];
for k=1:26
    path_xyzrpy(k,:) = pose2xyzrpy(path(:,:,k));
end
for i=1:6
    for j=1:25
        path_xyzrpy_dot(j,i) = (path_xyzrpy(j+1,i)-path_xyzrpy(j,i))/2;
    end
end
for i=1:3
    subplot(3,2,i);    
    plot(1:26, path_xyzrpy(:,i), 'b', 1:26, path_xyzrpy_dot(:,i), 'g');
    title(plot_titles(i));
    xlabel("steps");
    ylabel("m, m/s");
end
for i=4:6
    subplot(3,2,i);    
    plot(1:26, path_xyzrpy(:,i), 'b', 1:26, path_xyzrpy_dot(:,i), 'g');
    title(plot_titles(i));
    xlabel("steps");
    ylabel("deg, deg/s");
end

%% Send commands to V-REP remoteApiServer
if (clientID>-1)
    disp('Connected to remote API server');

    % Command UR10 robot    
    for i = 1:26
        disp("step: "+i);
        moveRobot(clientID, vrep, path_q(i,:));
        pause(5);
    end

else
    disp('Failed connecting to remote API server');
end


%% Close the connection to V-REP:  
if (clientID>-1)
    vrep.simxFinish(clientID);
end
vrep.delete(); % call the destructor!

disp('Program ended');