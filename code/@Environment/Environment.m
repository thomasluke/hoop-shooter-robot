classdef Environment < handle
    % Environment contains functions and parameters to build the environment
    
    properties (Access = private)
        
        ballPose;
        objectMesh_h=50;
        robot; % Control the robot of choice
        object; % object to build in environment
        hoopPose; % Hoop position and orientation
        
    end
    
    methods % Public
        %         function a = get.objectMesh_h(obj)
        %             a = obj.objectMesh_h;
        %         end
        %     end
        %
        
        function obj = Environment(robot,ballPose,hoopPose)
            
            obj.robot = robot;
            obj.ballPose.position = ballPose.position;
            obj.ballPose.orientation = ballPose.orientation; % Ball is a sphere and orientation does not matter
            obj.hoopPose.position = hoopPose.position;
            obj.hoopPose.orientation= hoopPose.orientation;
            obj.addBackgroundImage();
        end
        
        function [objectMesh_h] = LoadObject(obj,objectName, position, orientation)
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Modified from 41013 Robotics week 4 material material
            % "PuttingSimulatedObjectsIntoTheEnvironment.m"
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % Loads object into environment in specified position and orientation
            
            [f,v,data] = plyread(objectName,'tri');
            
            % Scale the colours to be 0-to-1 (they are originally 0-to-255
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            
            % Then plot the trisurf
            objectMesh_h = trisurf(f,v(:,1),v(:,2), v(:,3) ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
            
            % Get vertex count
            objectVertexCount = size(v,1);
            
            % Move center point to origin
            midPoint = sum(v)/objectVertexCount;
            objectVerts = v - repmat(midPoint,objectVertexCount,1);
            
            innacuracyOffset = 0.006518;
            
            position = [position(1)+midPoint(1)*cos(orientation),position(2)+midPoint(1)*sin(orientation),position(3)+midPoint(3)];
            
            % Move forwards (facing in -y direction)
            forwardTR = makehgtform('translate',position-innacuracyOffset);
            
            % % Random rotate about Z
            rotateTR = makehgtform('zrotate',orientation);
            
            objectPose = eye(4);
            
            % Move the pose forward and a slight and random rotation
            objectPose = objectPose * forwardTR *rotateTR;
            updatedPoints = [objectPose * [objectVerts,ones(objectVertexCount,1)]']';
            
            % Now update the Vertices
            objectMesh_h.Vertices = updatedPoints(:,1:3);
            drawnow();
            
        end
        
        function [objects] = BuildEnvironment(obj)

            doCameraSpin = false;
            
            hold on;
            
            axis([-7 7 -7 7 -0.01 7]);
            
            % Load table and safety fencing
%           Stop sign model: https://grabcad.com/library/priority-sign-207-1
%           Fire extinguisher model: https://grabcad.com/library/fire-extinguisher-1-kg-dutch-english-french-german-1-of-3-1            
            disp('Loading safety fence');
            safety_fence = obj.LoadObject("fencing.ply", [0,0,0],0);
            
%           Basketball hoop model: https://grabcad.com/library/basketball-hoop-10
            disp('Loading hoop');
            hoop = obj.LoadObject("hoop.ply", obj.hoopPose.position,obj.hoopPose.orientation);
            
            disp('Loading ball');
            ball = obj.LoadObject("basketball.ply", obj.ballPose.position,obj.ballPose.orientation);
            
            objects = {ball,hoop,safety_fence};
            
            %             % Set robot base positions so they are in the same spot on the table regardless of scene position or orientation
            %             ur3.model.base = ur3.model.base * transl(0.65*cos(sceneOrientation)+xScenePosition,0.75*sin(sceneOrientation)+yScenePosition,tableSurfaceHeight)*trotz(sceneOrientation);
            %             ur5.model.base = ur5.model.base* transl(0*sin(sceneOrientation)+yScenePosition,tableSurfaceHeight,0*cos(sceneOrientation)+xScenePosition)*troty(sceneOrientation); % y, z, x
            
            %             p = plot3(obj.hoopPose.position(1),obj.hoopPose.position(2),obj.hoopPose.position(3),'ro');
            %             p.MarkerSize=10
            %
        end
        
    end
    
    methods (Access = private)
        
        function addBackgroundImage(obj)
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Modified from Patrick Kalita on MATLAB answers
            % "RGB-images on a 3D cube" - https://www.mathworks.com/matlabcentral/answers/32070-rgb-images-on-a-3d-cube 
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % Image from: https://in.pinterest.com/pin/746964288181450369/
            cdata = flip( imread('basketball court - small.png'), 1 );
            cdatar = flip( cdata, 2 );
            
            surface([-7 7; -7 7], [-7 -7; 7 7], [0.01 0.01; 0.01 0.01], ...
                'FaceColor', 'texturemap', 'CData', cdata );
            
            view(3);
        end
        
    end
end
