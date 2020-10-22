classdef Control < handle
    % control contains functions and parameters to control robot movements
    
    properties (Access = private)
        
        robot; % Control the robot of choice
        object; % object to pick and throw
        hoop; % Stores the hoop mesh
        safetyFence; % Stores the safety fence mesh
        hoopPose; % Hoop position and orientation
        objectPosition; % Position of the pick object
        launchVelocity; % Initial launch velocity of the object
        moveVelocity; % Default velocity of RMRC movements
        launchAngle; % Angle of throw release
        allignmentAngle; % 360 degree angle for robot to allign to hoop.
        pullBackDirection; % 1 or 0. 1 if allignmentAngle is > 180 and < 360. This accounts for joint 1 not having full 360 degree range. Robot will just move between 180 degree range and bend back in different directions.
        projectilePath; % Trajecotry of the projectile motion
        endQ; % Desired robot joint config
        qMatrix; % Joint config trajectory
        collisionObjects; % Containers of all collisison objects (includes the ground)
        collisionPlot; % Plot of all points where collisions ar edetected alongs a trajectory
        trajectoryPlot; % Plot of RMRC trajectory/path
        avoidCollisions; % Sets whether to avoid collisions
        trajectoryLength; % Define number of positions for each move (higher number = slower but smoother movements)
        resolveMotionRateControl; % Swaps between RMRC (true) and inverse kinematics (false)
        errorMax; % Stores the maximum RMRC error (along x, y or z)
        objectPlot;
    end
    
    methods % Public
        
        function obj = Control(robot,robotQ,objects,hoopPose,launchAngle,projectilePath,intialVelocity,avoidCollisions,resolveMotionRateControl)
            
            obj.robot = robot;
            obj.object = objects{1}; % Ball
            obj.hoop = objects{2}; % Hoop
            obj.safetyFence = objects{3}; % Safety fence
            obj.hoopPose = hoopPose;
            obj.launchAngle = launchAngle;
            obj.projectilePath = projectilePath;
            obj.launchVelocity = intialVelocity;
            obj.collisionObjects = obj.CollisionObjects();
            obj.avoidCollisions = avoidCollisions;
            obj.trajectoryLength = 100;
            obj.resolveMotionRateControl = resolveMotionRateControl;
            obj.errorMax = 1;
            obj.moveVelocity = 2;
            
        end
        
        function ShootHoop(obj)
            
            % Moves the robot to pick up the object
            obj.Pick();
            
            % Alligns the robot to the throwing positions and launch the object
            obj.Throw();
            
        end
        
        function MoveObject(obj,object,position,orientation)
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Modified from 41013 Robotics week 4 material material
            % "PuttingSimulatedObjectsIntoTheEnvironment.m"
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % MoveObject moves object to a specified position and orientation
            
            v = object.Vertices;
            
            % Get vertex count
            objectVertexCount = size(v,1);
            
            % Move center point to origin
            midPoint = sum(v)/objectVertexCount;
            objectVerts = v - repmat(midPoint,objectVertexCount,1);
            
            deltaPosition = [position(1)-midPoint(1),  position(2)-midPoint(2), position(3)-(midPoint(3))]; % x, y, -z
            
            % Create a transform to describe the location (at the origin, since it's centered
            objectPose = eye(4);
            
            % Move forwards (facing in -y direction)
            forwardTR = makehgtform('translate',deltaPosition+midPoint);
            
            % % Random rotate about Z
            rotateTR = makehgtform('zrotate',orientation);
            
            % Move the pose forward and a slight and random rotation
            objectPose = objectPose * forwardTR*rotateTR;
            updatedPoints = [objectPose * [objectVerts,ones(objectVertexCount,1)]']';
            
            % Now update the Vertices
            object.Vertices = updatedPoints(:,1:3);
            drawnow();
            
        end
        
        
    end
    
    methods (Access = private)
        
        
        function Pick(obj)
            
            % Update objects position using mesh mid point
            obj.UpdateObjectPositon();
            
            launching = false;
            
            if obj.resolveMotionRateControl == true;
                
                % Move just above the object
                point = obj.objectPosition;
                point(3) = point(3) + 0.15;
                obj.MoveEndEffectorToPoint(point,false,obj.moveVelocity,launching);
                
                % Slide picker down over top of the object
                point = obj.objectPosition;
                point(3) = point(3) - 0.07;
                obj.MoveEndEffectorToPoint(point,false,obj.moveVelocity,launching);
                
                % Move picker to pick up object
                point = obj.objectPosition;
                obj.MoveEndEffectorToPoint(point,false,obj.moveVelocity,launching);
                
            elseif    obj.resolveMotionRateControl == false;
                
                % Move picker up to pick up object
                point = obj.objectPosition;
                obj.MoveEndEffectorToPoint(point,false,obj.moveVelocity,launching);
                
            end
            
        end
        
        function Throw(obj)
            
            if obj.resolveMotionRateControl == true;
                
                launching = false;
                
                % Move picker and object upwards so they dont collide with the ground
                point = obj.objectPosition;
                point(3) = point(3) + 0.1;
                moveObject = true;
                obj.MoveEndEffectorToPoint(point,moveObject,obj.moveVelocity,launching);
                
                % Calculate allignment angle to shoot the hoop
                obj.allignmentAngle =atan2(-obj.hoopPose.position(2)-obj.robot.model.base(2,4),-obj.hoopPose.position(1)-obj.robot.model.base(1,4));
                
                % Animate joint 1 to match the hoop shooting allignment angle
                jointNumber = 1;
                moveObject = true;
                obj.AnimateToJointConfig(jointNumber,moveObject,obj.allignmentAngle);
                
                % Move link to optimum throwing angle
                obj.AllignThrowAngle(deg2rad(90),0,0);
                
                q = [obj.allignmentAngle,obj.launchAngle,0,0,0,0,0];
                
                launchTransform = obj.robot.model.fkine(q);
                launchPoint = launchTransform(1:3,4)';
                
                launching = true;
                
                % Throw ball and release at the specified launch angle
                moveObject = true;
                obj.ResolveMotionRateControl(launchPoint,moveObject,obj.launchVelocity,launching);
                
                
            elseif obj.resolveMotionRateControl == false
                
                % Move link to optimum throwing angle
                obj.AllignThrowAngle(deg2rad(60),deg2rad(15),deg2rad(15));
                
                % Throw ball and release at the specified launch angle
                obj.AllignThrowAngle(obj.launchAngle,0,0);
                
            end
            
            % Move the ball along the projectile path
            for i=1:1:size(obj.projectilePath)
                
                obj.MoveObject(obj.object,obj.projectilePath(i,:),0);
                
            end
            
            % Turn to allign with hoop
            
            % Make sure the gripper stays horizontal to whole time so that
            % the ball does not "fall off" until throwing
            
            % set joint anlges to outstretch all links except
            % bend back link 1 90 degrees (robot laying
            % horizontal to ground)
            % Throw at a calculated speed and stop movement at the desired launch angle (ball
            % will coninue its motion
        end
        
        function AnimateToJointConfigTrajectory(obj,jointNumber,Q,CCW)
            
            startQ = obj.robot.model.getpos;
            q = startQ;
%             if startQ(2) == 0
%             startQ(2) = startQ(2)+eps; % Stops sign() from = 0;
%             end
%             Q = sign(startQ(2)) * Q;
            %             Q=pi-Q;
            if CCW == 0 % 0 = false = CW
                jointConfig = obj.robot.model.getpos;
                %                  Q = -Q+jointConfig(2);
                Q=pi+Q;
                
                
                disp('Checking for obstacles in counter clockwise trajectory');
            else
                jointConfig = obj.robot.model.getpos;
                %                 Q = Q+jointConfig(2);
                disp('Checking for obstacles in clockwise trajectory');
            end
            
            % Animation step increment size to have the number of steps equal obj.trajectoryLength
            increment = (Q-startQ(jointNumber))/obj.trajectoryLength;
            
            qArray = zeros(obj.trajectoryLength,size(q,2));
            row = 0;
            
            % Animate specified joint number to specified angle
            for i=startQ(jointNumber):increment:Q
                row = row+1;
                q (jointNumber) = i;
                qArray(row,:)=q;
                %
            end
            
            obj.qMatrix = qArray;
            
        end
        
        function AnimateToJointConfig(obj,jointNumber,moveObject,Q)
            
            while true % Loop until collision object moves
                
                %             pause(0.1); % Pauses added to remove code skipping error;
                
                % Number of collision objects created
                numberOfObjects = size(obj.collisionObjects,2);
                
                % Initialising results container
                results = ones(1,numberOfObjects);
                
                if obj.avoidCollisions == true
                    
                    % Initialising results container
                    counterClockwise = [false,true];
                    
                    jointConfig = obj.robot.model.getpos;
                    ccwQ = Q+2*pi;
                    cwQ = Q;
                    
                    if ccwQ > deg2rad(170) || ccwQ < deg2rad(-170)
                        counterClockwise(1) = [];
                        
                    end
                    if cwQ > deg2rad(170) || cwQ < deg2rad(-170)
                        counterClockwise(end) = [];
                        %                     else
                        %                         counterClockwise = [false,true];
                    end
                    
                    if size(counterClockwise,2) == 0
                        disp('SIMULATION PAUSED: Joint limit at launch angle');
                        %                             break;
                    end
                    
                    
                    for i=1:1:size(counterClockwise,2)
                        
                        AnimateToJointConfigTrajectory(obj,jointNumber,Q,counterClockwise(i));
                        
                        delete(obj.collisionPlot);
                        obj.collisionPlot = [];
                        
                        for j=1:1:numberOfObjects
                            
                            obstacle = obj.collisionObjects{j};
                            
                            results(j) = obj.CollisionDetection(obj.qMatrix,obstacle{1},obstacle{2},obstacle{3});
                            
                        end
                        
                        if ismember(1,results) == false
                            disp('Trajectory clear: moving along trajectory');
                            break;
                        end
                        
                    end
                    
                elseif obj.avoidCollisions == false
                    CW = true;
                    AnimateToJointConfigTrajectory(obj,jointNumber,Q,CW);
                end
                
                if ismember(1,results) == false || obj.avoidCollisions == false
                    for i=1:1:size(obj.qMatrix(:,1));
                        
                        
                        obj.robot.model.animate(obj.qMatrix(i,:));
                        
                        %               If true move object so it is always positioned on the robot end effector
                        if moveObject == true
                            obj.objectPosition=obj.robot.model.fkine(obj.robot.model.getpos);
                            obj.objectPosition = obj.objectPosition(1:3,4);
                            obj.MoveObject(obj.object,obj.objectPosition,0);
                        end
                    end
                    break; % Break out of the while loop after animating to joint config
                elseif ismember(1,results) == true && size(counterClockwise,2) > 0
                    disp('No clear trajectory: pausing robot until trajectory is cleared');
                    pause(5);
                    collisionObject = obj.collisionObjects{end};
                    delete(collisionObject{4});
                    obj.collisionObjects(end)=[];
                    %                 obj.collisionObjects(3) = [];
                end
            end
        end
        
        function AllignThrowAngle(obj,pullBackAngleLower,pullBackAngleMiddle,pullBackAngleUpper,launching)
            
            % Move the robot to the throw/allignment angle specified by the
            % projectile motion class
            
            % Calculate allignment angle to shoot the hoop
            obj.allignmentAngle =atan2(-obj.hoopPose.position(2)-obj.robot.model.base(2,4),-obj.hoopPose.position(1)-obj.robot.model.base(1,4));
            
            % Specify desired joint config
            obj.endQ = [obj.allignmentAngle,pullBackAngleLower,0,-pullBackAngleUpper,0,pullBackAngleMiddle,0];
            
            % Animate to specified joint config and move the object along the end effector
            moveObject = true;
            obj.MoveToJointConfig(obj.endQ,moveObject);
            
        end
        
        function MoveEndEffectorToPoint(obj,point,moveObject,velocity,launching)
            
            % Use inverse kinematics (ikcon)
            if obj.resolveMotionRateControl == false
                
                % Calculate the end pose joint config using inverse kinematics (ikcon)
                obj.CalculateQ(point);
                
                % Move to calculate Q joint configuration and move object if "moveObject == true""
                obj.MoveToJointConfig(obj.endQ,moveObject);
                
                % Use Resolve Motion Rate Control (RMRC)
            elseif obj.resolveMotionRateControl == true
                
                %  Move to specified point using RMRC
                obj.ResolveMotionRateControl(point,moveObject,velocity,launching);
                
            end
            
        end
        
        function CalculateQ(obj,point)
            
            % Calculating rotation of the end transform depending on object position
            if sign(point(2)) ~= 0
                rotationTransformX = sign(point(1))*-trotx(pi/2);
            else
                rotationTransformX = 1;
            end
            
            if sign(point(1)) ~= 0
                rotationTransformY = sign(point(1))*troty(pi/2);
            else
                rotationTransformY = 1;
            end
            
            % End tranformation for the end effector
            endTransform = transl(point)*rotationTransformY*rotationTransformX;
            
            % Use endTransform to find the joint orientations for the end position
            obj.endQ = obj.robot.model.ikcon(endTransform,obj.robot.model.getpos);
            
            endTransformCheck = obj.robot.model.fkine(obj.endQ);
            
            % Difference (absolute positive magnitude) between the specified end Transform and actual
            % end Tansform calculated by ikcon (inverse kinematics)
            transformError = abs(endTransformCheck) - abs(endTransform);
            
            % Position error
            translationError = transformError(1:3,4)';
            
            % Rotation error
            rotationError = rad2deg(tr2rpy(transformError(1:3,1:3)));
            
            % Display errors
            disp('Transformation Error: ')
            disp(transformError);
            
            disp('Translation Error: ')
            disp(translationError);
            
            disp('Rotation Error (degrees): ')
            disp(rotationError);
            
        end
        
        function MoveToJointConfig(obj,endQ,moveObject)
            
            
            % Finding the robot joint positions trajectory required to move the end effector to the end point
            obj.qMatrix = jtraj(obj.robot.model.getpos(),endQ,obj.trajectoryLength);
            
            if obj.avoidCollisions == true
                
                obj.resolveMotionRateControl = false;
                % Avoid collisions to reach the end point
                launching = false;
                obj.CollisionAvoidance(moveObject,launching);
                obj.resolveMotionRateControl = true;
                
            end
            
            % Animate a joint config trajectory
            obj.AnimateTrajectory(obj.qMatrix,moveObject);
            
        end
        
        function AnimateTrajectory (obj,trajectory,moveObject)
            
            % Iterate the robot arms through their movement
            for trajStep = 1:size(trajectory,1)
                
                Q = trajectory(trajStep,:);
                
                % calculate end effector position using fkine
                fkine = obj.robot.model.fkine(obj.robot.model.getpos());
                endEffectorPosition = fkine(1:3,4);
                
                if moveObject == true
                    % Move object to the end effector position for each
                    % trajectory step (simulates ball movement)
                    obj.MoveObject(obj.object,endEffectorPosition,0);
                end
                
                %                 plot3(endEffectorPosition(1),endEffectorPosition(2),endEffectorPosition(3),'k.','LineWidth',1);
                
                % Animate robot through a fraction of the total movement
                obj.robot.model.animate(Q);
                
                drawnow();
            end
        end
        
        function ResolveMotionRateControlCalculateTrajectory(obj,endPoint,velocity,launching)
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Modified from 41013 Robotics week 9 material
            % "Lab9Solution_Question1.m"
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            
            
            % Delete previous trajectory plot
            delete(obj.trajectoryPlot);
            obj.trajectoryPlot = [];
            
            % Calculate distance to point to scale the number of steps for
            % each trajectory
            robotTransform = obj.robot.model.fkine(obj.robot.model.getpos);
            startPoint = robotTransform(1:3,4)';
            distanceToEndPoint = norm(endPoint-startPoint);
            
            % Robotics
            % Lab 9 - Question 1 - Resolved Motion Rate Control in 6DOF
            % 1.1) Set parameters for the simulation
            % mdl_puma560;        % Load robot model
            t = (distanceToEndPoint)/velocity;             % Total time (s)
            deltaT = 0.005;      % Control frequency
            steps = round(t/deltaT,0,'decimals');   % No. of steps for simulation
            delta = 2*pi/steps; % Small angle change
            epsilon = 0.1;      % Threshold value for manipulability/Damped Least Squares
            W = diag([1 1 1 0.1 0.1 0.1]);    % Weighting matrix for the velocity vector 1s for more weighting than 0.1 angular velocities
            
            % 1.2) Allocate array data
            m = zeros(steps,1);             % Array for Measure of Manipulability
            obj.qMatrix = zeros(steps,7);       % Array for joint anglesR
            qdot = zeros(steps,7);          % Array for joint velocities
            theta = zeros(3,steps);         % Array for roll-pitch-yaw angles
            x = zeros(3,steps);             % Array for x-y-z trajectory
            positionError = zeros(3,steps); % For plotting trajectory error
            angleError = zeros(3,steps);    % For plotting trajectory error
            
            % Modify the rotation of the end effector based on the object position
            if endPoint(1)>0.01
                objectTransform = troty(-pi/2);
            elseif endPoint(1) <0.01
                objectTransform = troty(pi/2);
            else
                objectTransform = eye(4);
            end
            
            if endPoint(2)>0.01
                objectTransform = objectTransform*trotx(-pi/2);
            elseif endPoint(2) <0.01
                objectTransform = objectTransform*trotx(pi/2);
            end
            
            rpy = tr2rpy(objectTransform);
            
            % 1.3) Set up trajectory, initial pose
            s = lspb(0,1,steps);                % Trapezoidal trajectory scalar
            for i=1:steps
                x(1,i) = (1-s(i))*startPoint(1) + s(i)*endPoint(1); % Points in x
                x(2,i) = (1-s(i))*startPoint(2) + s(i)*endPoint(2); % Points in y
                if launching == true
                    x(3,i) = startPoint(3) + sqrt((hypot(startPoint(2),startPoint(1)))^2-((hypot(x(1,i),x(2,i))))^2);% Points in z
                elseif launching == false
                    x(3,i) = (1-s(i))*startPoint(3) + s(i)*endPoint(3); % Points in z
                    %     x(3,i) = endPoint(3) + 0.2*sin(i*delta); % Points in z
                    
                end
                test = 1;
                if velocity>3
                    test = -1;
                end
                theta(1,i) = test*rpy(1);                                        % Roll angle
                theta(2,i) = test*rpy(2);                                        % Pitch angle
                theta(3,i) = test*rpy(3);                                        % Yaw angle
            end
            
            q0 = zeros(1,7);                                                            % Initial guess for joint angles
            obj.qMatrix(1,:) = obj.robot.model.getpos;                                            % Solve joint angles to achieve first waypoint
            
            % 1.4) Track the trajectory with RMRC
            for i = 1:steps-1
                T = obj.robot.model.fkine(obj.qMatrix(i,:));                                           % Get forward transformation at current joint state
                deltaX = x(:,i+1) - T(1:3,4);                                         	% Get position error from next waypoint
                Rd = rpy2r(theta(1,i+1),theta(2,i+1),theta(3,i+1));                     % Get next RPY angles, convert to rotation matrix
                Ra = T(1:3,1:3);                                                        % Current end-effector rotation matrix
                Rdot = (1/deltaT)*(Rd - Ra);                                            % Calculate rotation matrix error
                S = Rdot*Ra';                                                           % Skew symmetric!
                linear_velocity = (1/deltaT)*deltaX;
                angular_velocity = [S(3,2);S(1,3);S(2,1)];                              % Check the structure of Skew Symmetric matrix!!
                deltaTheta = tr2rpy(abs(Rd*Ra'));                                            % Convert rotation matrix to RPY angles. Gives error as rotation matrix times its transform should equal the indentity matrix
                xdot = W*[linear_velocity;angular_velocity];                          	% Calculate end-effector velocity to reach next waypoint.
                J = obj.robot.model.jacob0(obj.qMatrix(i,:));                                          % Get Jacobian at current joint state. Jacob gives with respect to base!!!
                m(i) = sqrt(abs(det(J*J')));
                if m(i) < epsilon                                                       % If manipulability is less than given threshold (epsilon was set at top)
                    lambda = (1 - m(i)/epsilon)*5E-2; % If manipubility is insufficient then damping will be < 0
                else
                    lambda = 0; % If manipubility is sufficient then damping will be = 0
                end
                invJ = inv(J'*J + lambda *eye(7))*J';                                   % DLS Inverse
                qdot(i,:) = (invJ*xdot)';                                               % Solve the RMRC equation (you may need to transpose the         vector)
                for j = 1:7                                                             % Loop through joints 1 to 7
                    if obj.qMatrix(i,j) + deltaT*qdot(i,j) < obj.robot.model.qlim(j,1)                     % If next joint angle is lower than joint limit...
                        qdot(i,j) = 0; % Stop the motor
                    elseif obj.qMatrix(i,j) + deltaT*qdot(i,j) > obj.robot.model.qlim(j,2)                 % If next joint angle is greater than joint limit ...
                        qdot(i,j) = 0; % Stop the motor
                    end
                end
                obj.qMatrix(i+1,:) = obj.qMatrix(i,:) + deltaT*qdot(i,:);                         	% Update next joint state based on joint velocities
                positionError(:,i) = x(:,i+1) - T(1:3,4);                               % For plotting
                angleError(:,i) = deltaTheta;                                           % For plotting
            end
            
            Transfrom = obj.robot.model.fkine(obj.qMatrix(end,:));
            endPosition = Transfrom(1:3,4)';
            
            error = endPosition-endPoint;
            obj.errorMax = max(abs(error));
            
            obj.trajectoryPlot = plot3(x(1,:),x(2,:),x(3,:),'k.','LineWidth',1);
            
            % Display errors
            disp('End Point Translation Error: ')
            disp(error);
            
            disp('End Point Maximum Translation Error: ')
            disp(obj.errorMax);
            
            disp('End Point Rotation Error (degrees): ')
            disp(rad2deg(angleError(:,end))');
            
        end
        
        function [q,point] = GenerateRandomQAndPoint (obj)
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Modified from 41013 Robotics week 5 material
            % "Lab5Solution_Question2and3.m"
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % Generate random joint config and end position
            
            qRand = (2 * rand(1,7) - 1) * pi;
            
            q = qRand;
            
            transform = obj.robot.model.fkine(q);
            point = transform(1:3,4)';
            
            while hypot(point(1),point(2)) < 0.2
                
                qRand = (2 * rand(1,7) - 1) * pi;
                
                q = qRand;
                
                transform = obj.robot.model.fkine(q);
                point = transform(1:3,4)';
                
            end
            
            
        end
        
        %         function ResolveMotionRateControlErrorControl(obj,endPoint)
        %
        %             obj.ResolveMotionRateControlCalculateTrajectory(endPoint);
        %
        %             while obj.errorMax > 0.05
        %                 while obj.errorMax > 0.05
        %                     [q,point]=obj.GenerateRandomQAndPoint();
        %                     obj.ResolveMotionRateControlCalculateTrajectory(point);
        %                 end
        %
        %                 if obj.avoidCollisions == true
        %                     obj.CollisionAvoidance();
        %                 end
        %
        %                 obj.AnimateTrajectory(obj.qMatrix,false);
        %
        %                 obj.ResolveMotionRateControlCalculateTrajectory(endPoint);
        %
        %
        %             end
        %
        %         end
        
        function ResolveMotionRateControl (obj,endPoint,moveObject,velocity,launching)
            
            % Calculate RMRC trajectory
            obj.ResolveMotionRateControlCalculateTrajectory(endPoint,velocity,launching);
            
            %             obj.ResolveMotionRateControlErrorControl(endPoint);
            if obj.avoidCollisions==true
                % Avoid collisions to reach the end point
                obj.CollisionAvoidance(moveObject,launching);
            end
            
            obj.AnimateTrajectory(obj.qMatrix,moveObject);
            
            delete(obj.trajectoryPlot);
            obj.trajectoryPlot = [];
            
        end
        
        function CollisionAvoidance(obj,moveObject,launching)
            
            % Number of collision objects created
            numberOfObjects = size(obj.collisionObjects,2);
            
            % Initialising results container
            results = zeros(1,numberOfObjects);
            
            % End joint config
            obj.endQ = obj.qMatrix(end,:);
            q = obj.endQ;
            
            while true  % Loop until possible to move to end q without collisions
                
                %                 if obj.resolveMotionRateControl == false
                
                if q ~= obj.endQ | obj.resolveMotionRateControl == false
                    % Finding the robot joint positions required to move the end effector to the end point
                    trajectory = jtraj(obj.robot.model.getpos(),q,obj.trajectoryLength);
                elseif q == obj.endQ & obj.resolveMotionRateControl == true
                    
                    velocity = obj.moveVelocity;
                    endFkine = obj.robot.model.fkine(q);
                    endPoint = endFkine(1:3,4)';
                    obj.ResolveMotionRateControlCalculateTrajectory(endPoint,velocity,launching);
                    trajectory = obj.qMatrix();
                end
                %                 elseif obj.resolveMotionRateControl == true
                %
                %                     % Move to endPoint using RMRC
                %                     transform = obj.robot.model.fkine(q);
                %                     endPoint = transform(1:3,4)';
                %                     obj.ResolveMotionRateControlCalculateTrajectory(endPoint,obj.moveVelocity);
                %
                %                 end
                
                delete(obj.collisionPlot);
                obj.collisionPlot = [];
                
                % Check for trajectory collision before animating for each obstacle
                for i=1:1:numberOfObjects
                    
                    obstacle = obj.collisionObjects{i};
                    
                    results(i) = obj.CollisionDetection(trajectory,obstacle{1},obstacle{2},obstacle{3});
                    
                end
                
                % If collision is along trajectory calculate a random new trajectory
                if ismember(1,results)
                    
                    disp('Collision along trajectory: recalculating alternate trajectory');
                    
                    % Calculate a random joint config and its end effector endPoint
                    [q,endPoint] = obj.GenerateRandomQAndPoint();
                    
                    %                     qRand = (2 * rand(1,7) - 1) * pi;
                    %
                    %                     q = qRand;
                    %
                    %                     transform = obj.robot.model.fkine(q);
                    %                     endPoint = transform(1:3,4)';
                    
                    %                     if  obj.resolveMotionRateControl == true
                    %                         while sqrt(endPoint(1)^2+endPoint(2)^2+endPoint(3)^2)<0.8 || sqrt(endPoint(1)^2+endPoint(2)^2+endPoint(3)^2)>1.5 || endPoint(3)>0.4
                    %
                    %                             qRand = (2 * rand(1,7) - 1) * pi;
                    %
                    %                             q = qRand;
                    %
                    %                             transform = obj.robot.model.fkine(q);
                    %                             endPoint = transform(1:3,4)';
                    %
                    %                         end
                    
                    
                else
                    
                    % Move to random pose that is not in collision. Set q to equal the goal end Q and check if it is now in collision
                    if q ~= obj.endQ
                        
                        disp('Moving to random pose that is not in collision');
                        
                        obj.AnimateTrajectory(trajectory,moveObject);
                        
                        q = obj.endQ;
                        
                    else % if there is no collisions to move toward the final q pose
                        
                        disp('No collisions along trajectory: moving to target pose');
                        
                        %                         endQ = obj.endQ;
                        
                        break;
                    end
                    
                end
                
                % Clear result collision checker
                results = [];
                
            end
            
        end
        
        
        function [collisionObjects] = CollisionObjects(obj)
            
            % Define collision objects
            
            % Floor
            
            centerpnt = [0,0,-2.51];
            %             side = 6;
            height = 5;
            plotOptions.plotFaces = true;
            colour = false;
            
            [vertex,faces,faceNormals,patch] = obj.RectangularPrism(centerpnt-height/2, centerpnt+height/2,plotOptions,colour);
            
            floor = {vertex,faces,faceNormals,patch};
            
            % Obstacle box
            
            centerpnt = [-1,1,0.5];
            %             centerpnt = [1,1,0.5];
            
            %             side = 6;
            height = 1;
            plotOptions.plotFaces = true;
            colour = true;
            
            [vertex,faces,faceNormals,patch] = obj.RectangularPrism(centerpnt-height/2, centerpnt+height/2,plotOptions,colour);
            
            obstacleBox = {vertex,faces,faceNormals,patch};
            
            centerpnt = [1,1,0.5];
            %             centerpnt = [1,1,0.5];
            
            %             side = 6;
            height = 1;
            plotOptions.plotFaces = true;
            colour = true;
            
            [vertex,faces,faceNormals,patch] = obj.RectangularPrism(centerpnt-height/2, centerpnt+height/2,plotOptions,colour);
            
            obstacleBox2 = {vertex,faces,faceNormals,patch};
            %              delete(obstacleBox2{4});
            collisionObjects = {floor,obstacleBox,obstacleBox2};
            
        end
        
        function result = CollisionDetection(obj,qMatrix,vertex,faces,faceNormals,returnOnceFound)
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Modified from 41013 Robotics week 5 material
            % "Lab5Solution_Question2and3.m"
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % Given a robot model (robot), and trajectory (i.e. joint state vector) (qMatrix)
            % and triangle obstacles in the environment (faces,vertex,faceNormals)
            
            if nargin < 6
                returnOnceFound = true;
            end
            result = false;
            
            row = 1;
            collisionPoints = zeros(row,3);
            
            for qIndex = 1:size(qMatrix,1)
                % Get the transform of every joint (i.e. start and end of every link)
                tr = obj.GetLinkPoses(qMatrix(qIndex,:));
                
                % Go through each link and also each triangle face
                for i = 1 : size(tr,3)-1
                    for faceIndex = 1:size(faces,1)
                        vertOnPlane = vertex(faces(faceIndex,1)',:);
                        [intersectP,check] = obj.LinePlaneIntersection(faceNormals(faceIndex,:),vertOnPlane,tr(1:3,4,i)',tr(1:3,4,i+1)');
                        if check == 1 && obj.IsIntersectionPointInsideTriangle(intersectP,vertex(faces(faceIndex,:)',:))
                            
                            collisionPoints(row,:) = intersectP;
                            row = row + 1;
                            
                            result = true;
                            
                            %                             if returnOnceFound
                            %                                                                 return
                            %                             end
                        end
                    end
                end
            end
            
            numberOfIntersections = size(collisionPoints);
            numberOfIntersections = numberOfIntersections(end,1);
            
            collisionPlotSize = size(obj.collisionPlot);
            collisionPlotSize = collisionPlotSize(end,1);
            
            for i=1:1:numberOfIntersections
                obj.collisionPlot(i+collisionPlotSize,:) = plot3(collisionPoints(i,1),collisionPoints(i,2),collisionPoints(i,3),'r*');
            end
            
        end
        
        function [ transforms ] = GetLinkPoses(obj, q)
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Slightly Modified from 41013 Robotics week 5 material
            % "Lab5Solution_Question2and3.m"
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            links = obj.robot.model.links;
            transforms = zeros(4, 4, length(links) + 1);
            transforms(:,:,1) = obj.robot.model.base;
            
            for i = 1:length(links)
                L = links(1,i);
                
                current_transform = transforms(:,:, i);
                
                
                
                current_transform = current_transform * trotz(q(1,i) + L.offset) * ...
                    transl(0,0, L.d) * transl(L.a,0,0) * trotx(L.alpha);
                transforms(:,:,i + 1) = current_transform;
                
                %         position = current_transform(1:3,4);
                %         orientation = current_transform(1:3,1:3);
                %         orientation = tr2rpy(orientation)
                %
                %         radius = [0.136/2,0.136/2,0.18252/2];
                %
                %         [X,Y,Z] = ellipsoid( position(1), position(2), position(3), radius(1), radius(2), radius(3) );
                %         ellipsoidCollisionBox = surf(X,Y,Z);
                %         rotate(ellipsoidCollisionBox,orientation,45);
                
            end
        end
        
        function [intersectionPoint,check] = LinePlaneIntersection(obj,planeNormal,pointOnPlane,point1OnLine,point2OnLine)
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % From 41013 Robotics week 5 material
            % "LinePlaneIntersection.m"
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % Given a plane (normal and point) and two points that make up another line, get the intersection
            % Check == 0 if there is no intersection
            % Check == 1 if there is a line plane intersection between the two points
            % Check == 2 if the segment lies in the plane (always intersecting)
            % Check == 3 if there is intersection point which lies outside line segment
            
            intersectionPoint = [0 0 0];
            u = point2OnLine - point1OnLine;
            w = point1OnLine - pointOnPlane;
            D = dot(planeNormal,u);
            N = -dot(planeNormal,w);
            check = 0; %#ok<NASGU>
            if abs(D) < 10^-7        % The segment is parallel to plane
                if N == 0           % The segment lies in plane
                    check = 2;
                    return
                else
                    check = 0;       %no intersection
                    return
                end
            end
            
            %compute the intersection parameter
            sI = N / D;
            intersectionPoint = point1OnLine + sI.*u;
            
            if (sI < 0 || sI > 1)
                check= 3;          %The intersection point  lies outside the segment, so there is no intersection
            else
                check=1;
            end
        end
        
        function result = IsIntersectionPointInsideTriangle(obj,intersectP,triangleVerts)
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % From 41013 Robotics week 5 material
            % "Lab5Solution_Question2and3.m"
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % Given a point which is known to be on the same plane as the triangle
            % determine if the point is
            % inside (result == 1) or
            % outside a triangle (result ==0 )
            
            u = triangleVerts(2,:) - triangleVerts(1,:);
            v = triangleVerts(3,:) - triangleVerts(1,:);
            
            uu = dot(u,u);
            uv = dot(u,v);
            vv = dot(v,v);
            
            w = intersectP - triangleVerts(1,:);
            wu = dot(w,u);
            wv = dot(w,v);
            
            D = uv * uv - uu * vv;
            
            % Get and test parametric coords (s and t)
            s = (uv * wv - vv * wu) / D;
            if (s < 0.0 || s > 1.0)        % intersectP is outside Triangle
                result = 0;
                return;
            end
            
            t = (uv * wu - uu * wv) / D;
            if (t < 0.0 || (s + t) > 1.0)  % intersectP is outside Triangle
                result = 0;
                return;
            end
            
            result = 1;                      % intersectP is in Triangle
        end
        
        function [vertex,face,faceNormals,collisionObject] = RectangularPrism(obj,lower,upper,plotOptions,colour,axis_h)
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % From 41013 Robotics week 5 material
            % "RectangularPrism.m"
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            if nargin<4
                axis_h=gca;
                if nargin<3
                    plotOptions.plotVerts=false;
                    plotOptions.plotEdges=true;
                    plotOptions.plotFaces=true;
                end
            end
            hold on
            
            vertex(1,:)=lower;
            vertex(2,:)=[upper(1),lower(2:3)];
            vertex(3,:)=[upper(1:2),lower(3)];
            vertex(4,:)=[upper(1),lower(2),upper(3)];
            vertex(5,:)=[lower(1),upper(2:3)];
            vertex(6,:)=[lower(1:2),upper(3)];
            vertex(7,:)=[lower(1),upper(2),lower(3)];
            vertex(8,:)=upper;
            
            face=[1,2,3;1,3,7;
                1,6,5;1,7,5;
                1,6,4;1,4,2;
                6,4,8;6,5,8;
                2,4,8;2,3,8;
                3,7,5;3,8,5;
                6,5,8;6,4,8];
            
            if 2 < nargout
                faceNormals = zeros(size(face,1),3);
                for faceIndex = 1:size(face,1)
                    v1 = vertex(face(faceIndex,1)',:);
                    v2 = vertex(face(faceIndex,2)',:);
                    v3 = vertex(face(faceIndex,3)',:);
                    faceNormals(faceIndex,:) = unit(cross(v2-v1,v3-v1));
                end
            end
            % If plot verticies
            if isfield(plotOptions,'plotVerts') && plotOptions.plotVerts
                for i=1:size(vertex,1);
                    plot3(vertex(i,1),vertex(i,2),vertex(i,3),'r*');
                    text(vertex(i,1),vertex(i,2),vertex(i,3),num2str(i));
                end
            end
            
            % If you want to plot the edges
            if isfield(plotOptions,'plotEdges') && plotOptions.plotEdges
                links=[1,2;
                    2,3;
                    3,7;
                    7,1;
                    1,6;
                    5,6;
                    5,7;
                    4,8;
                    5,8;
                    6,4;
                    4,2;
                    8,3];
                
                for i=1:size(links,1)
                    plot3(axis_h,[vertex(links(i,1),1),vertex(links(i,2),1)],...
                        [vertex(links(i,1),2),vertex(links(i,2),2)],...
                        [vertex(links(i,1),3),vertex(links(i,2),3)],'k')
                end
            end
            
            % If you want to plot the edges
            if isfield(plotOptions,'plotFaces') && plotOptions.plotFaces
                
                tcolor = [.2 .2 .8];
                
                if colour == true
                    
                    transparency = 1; % 1 = 100% opaque
                    
                else
                    
                    transparency = 0; % 0 = 100% transparent
                    
                end
                
                collisionObject = patch('Faces',face,'Vertices',vertex,'FaceVertexCData',tcolor,'FaceColor','flat','FaceAlpha',transparency,'lineStyle','none');
            end
        end
        
        function UpdateObjectPositon(obj)
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Modified from 41013 Robotics week 4 material
            % "PuttingSimulatedObjectsIntoTheEnvironment.m"
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            v = obj.object.Vertices;
            
            % Get vertex count
            objectVertexCount = size(v,1);
            
            % Move center point to origin
            midPoint = sum(v)/objectVertexCount;
            
            %Remove inaccuracy from midPoint calculations when object should
            %be at 0
            
            obj.objectPosition = midPoint+0.0065;
            
            sizeObjectPosition = size(obj.objectPosition);
            sizeObjectPosition = sizeObjectPosition(2);
            
            for i=1:1:sizeObjectPosition
                
                if abs(obj.objectPosition(i)) <= 0.001
                    obj.objectPosition(i) = 0;
                end
            end
        end
        
        
    end
end