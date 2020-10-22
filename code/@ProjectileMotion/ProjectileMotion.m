classdef ProjectileMotion < handle
    % control contains functions and parameters to control robot movements
    
    properties (Access = private)
        
        robot;
        hoopPose;
        objectReleasePosition;
        
        velocity;
        launchAngle;
        allignmentAngle; % 360 degree angle for robot to allign to hoop. Positive x axis = 0 degrees
        
        gravity = -9.81 % m/s
        
        deltaPosition;
        
        initialVelocity;
        
        flightTime;
        
        velocityInitialHoriz;
        velocityFinalHoriz;
        velocityInitialVert;
        velocityFinalVert;
        
        xPosition;
        yPosition;
        zPosition;
        
        projectilePathPoint;
        projectilePath;
        plot;
        startPoint;
        endPoint;
        
    end
    
    methods % Public
        
        function obj = ProjectileMotion(robot,hoopPose,launchAngle)
            
            obj.robot = robot;
            obj.hoopPose = hoopPose; 
            obj.hoopPose.position = obj.hoopPose.position + eps; % add small error (eps), otherwise trajectory plot does not work if hoop pose for x,y or z = 0 
            obj.launchAngle = launchAngle;
            obj.allignmentAngle = atan2(hoopPose.position(2),hoopPose.position(1));
            
        end
        
        function [projectilePath,intialVelocity] = TrajectoryGenerator(obj)
            
            obj.Calculations();
            obj.PlotTrajectory();
            
            obj.projectilePath(isnan(obj.projectilePath))=0;
            
            projectilePath = obj.projectilePath;
            intialVelocity = obj.initialVelocity;
            
        end
        
    end
    
    methods (Access = private)
        
        function RobotReleasePosition(obj,pullBackAngleLower,pullBackAngleMiddle,pullBackAngleUpper)
            
            % Move the robot to the throw/allignment angle specified by the
            % projectile motion class
            
            % Account for robot base so that allignmentAngle is with respect to the base
            obj.allignmentAngle = atan2(obj.hoopPose.position(2)-obj.robot.model.base(2,4),obj.hoopPose.position(1)-obj.robot.model.base(1,4));
            
            allignmentQ = [obj.allignmentAngle,-pullBackAngleLower,0,pullBackAngleUpper,0,-pullBackAngleMiddle,0];
            
            robotFkine = obj.robot.model.fkine(allignmentQ);
            obj.objectReleasePosition = robotFkine(1:3,4)';
                        
        end
        
        function Calculations(obj)
            
            obj.RobotReleasePosition(obj.launchAngle,0,0);
            
            obj.deltaPosition = [obj.hoopPose.position(1)-obj.objectReleasePosition(1),obj.hoopPose.position(2)-obj.objectReleasePosition(2), obj.hoopPose.position(3)-obj.objectReleasePosition(3)];
            deltaHorizVert = [hypot(obj.deltaPosition(1),obj.deltaPosition(2)),obj.deltaPosition(3)];
            
            obj.initialVelocity = sqrt((obj.gravity*(deltaHorizVert(1))^2)/(2*(deltaHorizVert(2)-deltaHorizVert(1))*(cos(obj.launchAngle)*sin(obj.launchAngle))));

            disp(['launch velocity: ',num2str(obj.initialVelocity),' m/s']);
            
            obj.flightTime = deltaHorizVert(1)/(obj.initialVelocity*cos(obj.launchAngle)); % xf = x0 + v0*t + 1/2*a*t^2 -> t = xf-x0/v0
            
            obj.velocityInitialHoriz = obj.initialVelocity*cos(obj.launchAngle);
            obj.velocityFinalHoriz = obj.initialVelocity*cos(obj.launchAngle); % vf = v0 + at
            obj.velocityInitialVert = obj.initialVelocity*sin(obj.launchAngle); % vf = v0 + at
            obj.velocityFinalVert = obj.initialVelocity*sin(obj.launchAngle)+obj.gravity*obj.flightTime; % vf = v0 + at
            
            
            % Plot the throw trajectory in 3D space
            obj.PlotTrajectory();
            
        end
        
        function PlotTrajectory(obj)
            
            % Clear any existing projectile motion plot before replotting
            
            delete(obj.plot);
            obj.plot = [];
            
            delete(obj.startPoint);
            obj.startPoint = [];
            
            delete(obj.endPoint);
            obj.endPoint = [];
            
            obj.startPoint = plot3(obj.objectReleasePosition(1),obj.objectReleasePosition(2),obj.objectReleasePosition(3),'mo');
            obj.startPoint.MarkerSize=10;
           
            obj.endPoint = plot3(obj.hoopPose.position(1),obj.hoopPose.position(2),obj.hoopPose.position(3),'ro'); 
            obj.endPoint.MarkerSize=10;
            
            incrementSize = 0.01;
            
            zerosRows = floor(obj.flightTime/incrementSize);
            zerosSize= [zerosRows,3];
            
            obj.projectilePath = zeros(zerosSize);
            
            for t=0:incrementSize:obj.flightTime
                
                horizontalDistance = 0.5*(obj.velocityFinalHoriz+obj.velocityInitialHoriz)*t; % xf = x0 + 1/2(vf + v0)*t
                
                % Signs used to make sign match obj.deltaPosition so that
                % projectile motion arc points in the correct direction
                obj.xPosition = obj.objectReleasePosition(1) + horizontalDistance * cos(obj.allignmentAngle)*(sign(obj.deltaPosition(1))/sign(horizontalDistance * cos (obj.allignmentAngle)));
                obj.yPosition = obj.objectReleasePosition(2) + horizontalDistance * sin (obj.allignmentAngle)*(sign(obj.deltaPosition(2))/sign(horizontalDistance * sin (obj.allignmentAngle)));
                
                velocityCurrent = obj.velocityInitialVert + obj.gravity*t;
                
                verticalDistance = 0.5*(velocityCurrent+obj.velocityInitialVert)*t;
                
                obj.zPosition = obj.objectReleasePosition(3) + verticalDistance;
                
                hold on
                
                index = round(((t/incrementSize)+1),0,'decimals'); % Round to stop round errors (e.g. 15.0000 should be an integer 15 for indexing)
                
                obj.projectilePathPoint = [obj.xPosition,obj.yPosition,obj.zPosition];
                obj.projectilePath(index,:) = obj.projectilePathPoint;
%                 obj.collisionPlot(i+collisionPlotSize,:)
                obj.plot(index)= plot3(obj.xPosition,obj.yPosition,obj.zPosition,'b.');
%                 obj.plot{index}.MarkerSize=10;
                
            end
        end
        
        
    end
end
