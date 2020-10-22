clf;
clear all;

% Creates a log of the command window and clears any previous logs
dfile ='CommandWindowLog';
% if exist(dfile, 'file') ; delete(dfile); end
diary(dfile)
diary on

hold on;

kuka = LBRIIWAR800(0);
kuka.model.teach();
% kuka.model.base = kuka.model.base * trotz(pi/2);
% Orient the plot/workspace viewport
az = -45; % azimuth, az, is the horizontal rotation about the z-axis as measured in degrees from the negative y-axis. Positive values indicate counterclockwise rotation of the viewpoint
el = 15; % el is the vertical elevation of the viewpoint in degrees. Positive values of elevation correspond to moving above the object; negative values correspond to moving below the object.
view(az,el);

hoopPose.position = [5,4,3];
hoopPose.orientation = 0;

ballHeight = 0.12065; % 0.12065 = radius of NBA basketball

ballPose.position = [0,2,ballHeight];
ballPose.orientation = 0; % Ball is a sphere and orientation does not matter

environment = Environment(kuka,ballPose,hoopPose);

camlight;   

objects = environment.BuildEnvironment();

launchAngle = pi/4;

robotStartQ = [0,-pi/4,0,0,0,0,0]; % Reasonable initial guess for ikcon in control

kuka.model.animate([robotStartQ])

projectileMotion = ProjectileMotion(kuka,hoopPose,launchAngle);
[projectilePath,intialVelocity] = projectileMotion.TrajectoryGenerator();

avoidCollisions = true; % Toggle collision avoidance on and off
resolvedMotionRateControl = true; % inverse kinematics (quintic polynomials) = false resolved motion rate control (RMRC) = true

control = Control(kuka,robotStartQ,objects,hoopPose,launchAngle,projectilePath,intialVelocity, avoidCollisions, resolvedMotionRateControl);
control.ShootHoop();

% UR5Pointcloud(kuka,control,ballPose.position);