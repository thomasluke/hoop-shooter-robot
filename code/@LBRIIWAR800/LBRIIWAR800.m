
%%LBRIIWAR800 Model
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% 41013 Robotics
% Thomas Harrison
% August 2020
% Modified from Peter Corke's Robotics Toolbox for MATLAB
% "LinearUR5.m"
% Kuka LBR IIWA 7 R800 3D model from Kuka's official website: 
% https://www.kuka.com/en-de/services/downloads?terms=Language:en:1;product_name:LBR%20iiwa%207%20R800;cad_type:STEP;&q=

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef LBRIIWAR800 < handle
    properties
        
        model
        
        % Set the size of the workspace when drawing the robot
        workspace = [-7 7 -7 7 0 7];
        
        % Flag to indicate if gripper is used
        useGripper = false;
    
    end
    
    methods
        
function self = LBRIIWAR800(useGripper)
    if nargin < 1
        useGripper = false;
    end
    self.useGripper = useGripper;
    

self.GetLBRIIWAR800Robot();

self.PlotAndColourRobot();

end

%% GetLBRIIWAR800Robot
% Given a name (optional), create and return a LBRIIWAR800 robot model
function GetLBRIIWAR800Robot(self)
%     if nargin < 1
        % Create a unique name (ms timestamp after 1ms pause)
        pause(0.001);
        name = ['LBR_IIWA_R800_',datestr(now,'yyyymmddTHHMMSSFFF')];
%     end

L(1) = Link([0 0.34 0 -pi/2]); % 0.33997

L(2) = Link([0 0 0 pi/2]);

L(3) = Link([0 0.4 0 pi/2]);

L(4) = Link([0 0 0 -pi/2]);

L(5) = Link([0 0.4 0 -pi/2]); % 0.39998

L(6) = Link([0 0 0 pi/2]);

L(7) = Link([0 1.1997 0 0]);

 % Incorporate joint limits
    L(1).qlim = [-170 170]*pi/180;
    L(2).qlim = [-120 120]*pi/180;
    L(3).qlim = [-170 170]*pi/180;
    L(4).qlim = [-120 120]*pi/180;
    L(5).qlim = [-170 170]*pi/180;
    L(6).qlim = [-120 120]*pi/180;
    L(7).qlim = [-175 175]*pi/180;
% 
%     L(2).offset = -pi/2;
%     L(4).offset = -pi/2;
    
    self.model = SerialLink(L,'name',name);

end

%% PlotAndColourRobot
% Given a robot index, add the glyphs (vertices and faces) and
% colour them in if data is available 
function PlotAndColourRobot(self)%robot,workspace)
    for linkIndex = 0:self.model.n
        if self.useGripper && linkIndex == self.model.n
            [ faceData, vertexData, plyData{linkIndex+1} ] = plyread(['LBRIIWAR800Link',num2str(linkIndex),'Gripper.ply'],'tri'); %#ok<AGROW>
        else
            [ faceData, vertexData, plyData{linkIndex+1} ] = plyread(['LBRIIWAR800Link',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>
        end
        self.model.faces{linkIndex+1} = faceData;
        self.model.points{linkIndex+1} = vertexData;
    end

    % Display robot
    self.model.plot3d(zeros(1,self.model.n),'noarrow','workspace',self.workspace);
    if isempty(findobj(get(gca,'Children'),'Type','Light'))
        camlight
    end  
    self.model.delay = 0;

    % Try to correctly colour the arm (if colours are in ply file data)
    for linkIndex = 0:self.model.n
        handles = findobj('Tag', self.model.name);
        h = get(handles,'UserData');
        try 
            h.link(linkIndex+1).Children.FaceVertexCData = [plyData{linkIndex+1}.vertex.red ...
                                                          , plyData{linkIndex+1}.vertex.green ...
                                                          , plyData{linkIndex+1}.vertex.blue]/255;
            h.link(linkIndex+1).Children.FaceColor = 'interp';
        catch ME_1
            disp(ME_1);
            continue;
        end
    end
end        
    end
end