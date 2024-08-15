clear all;close all;clc

% ROS network connection: insert "rosinit" in your command window

% load your refernece path here (must be n by 2 arrary)
load path
path = yourpath;


% Create ROS publishers and subscribers for relevant topics
cmdVelPublisher = rospublisher("/pedbot/control/cmd_vel","DataFormat","struct");
poseSubscriber = rossubscriber('/pedsim_simulator/robot_position');
threhold_sub = rossubscriber('/signal_safe', 'std_msgs/Int32');


% Create a ROS message for velocity commands
cmdVelMsg = rosmessage(cmdVelPublisher);

% % Set the loop rate (adjust as needed)
loopRate = robotics.Rate(10); % 10 Hz

% Robot parameters
maxLinearVelocity = 0.5;  % Maximum linear velocity (adjust as needed)
maxAngularVelocity = 0.6; % Maximum angular velocity (adjust as needed)
wheelbase = 0.5;          % Distance between wheels (adjust as needed)


% Pure Pursuit Algorithm
controller = controllerPurePursuit;
controller.Waypoints = path; % Set the waypoints
controller.DesiredLinearVelocity = maxLinearVelocity;
controller.MaxAngularVelocity = maxAngularVelocity;
controller.LookaheadDistance = 5; % Adjust the lookahead distance as needed
forward_velocity_all = [];
tic
% Control loomatlab.matp
while true
    % Get the current robot pose from the subscriber

    pos = receive(poseSubscriber);
    xr_s=pos.Pose.Pose.Position.X;
    yr_s=pos.Pose.Pose.Position.Y;
    X_q=pos.Pose.Pose.Orientation.X
    Y_q=pos.Pose.Pose.Orientation.Y
    Z_q=pos.Pose.Pose.Orientation.Z
    W_q=pos.Pose.Pose.Orientation.W

    
    % Extract linear velocity components
    vx = pos.Twist.Twist.Linear.X;
    vy = pos.Twist.Twist.Linear.Y;
    vz = pos.Twist.Twist.Linear.Z;

    % Calculate the magnitude (forward velocity)
    forward_velocity = norm([vx, vy, vz]);
    forward_velocity_all = [forward_velocity_all,forward_velocity];
    
    quat=[W_q X_q Y_q Z_q ];
    eul = quat2eul(quat)
    heading_angle=eul(1)
    currentPose_withheading = [xr_s, yr_s,heading_angle];
    currentPose = [xr_s, yr_s];

    %Compute the desired steering angle (angular velocity)
    [linearVel, angularVel] = controller(currentPose_withheading);
  
       signal_safe = receive(threhold_sub);
       display(signal_safe);

       if  signal_safe.Data == 1
        cmdVelMsg.linear.x = 0;
        cmdVelMsg.angular.z = 0;

       elseif signal_safe.Data == 0

        % Set the computed velocity commands
        cmdVelMsg.linear.x = linearVel;
        cmdVelMsg.angular.z = angularVel;
      end
   
    % Publish the velocity commands
    send(cmdVelPublisher, cmdVelMsg);

    % Check if the goal is reached (you can define your own criteria)
    if norm(currentPose - path(end, :)) < 1
        cmdVelMsg.linear.x = 0;
        cmdVelMsg.angular.z = 0;
        send(cmdVelPublisher, cmdVelMsg);
        break; % Goal reached
    end100
    
    end

%     %Wait for the next iteration
%     waitfor(loopRate);

end
toc

average_velocity_forward = mean(forward_velocity_all);



