function[bricks] = BuildEnvironment(ur3,ur5,brickPositions,scenePosition, sceneOrientation)
% BuildEnvironment loads, positions and orients all the objects in the environment

% tableSurfaceHeight = 1

% Parameters to adust scene to any positions or orientation
tableSurfaceHeight = scenePosition(1,3);
heightAdjustment = scenePosition(1,3)-0.5;
xScenePosition = scenePosition(1,1);
yScenePosition = scenePosition(1,2);

% sceneOrientation = pi/2

doCameraSpin = false;

hold on

axis([-2.5+xScenePosition 2.5+xScenePosition -2.5+yScenePosition 2.5+yScenePosition 0+heightAdjustment 2.5+heightAdjustment])

% Load table and safety fencing

disp('Loading table');
LoadObject("table.ply",[xScenePosition,yScenePosition,heightAdjustment],sceneOrientation+0);

% disp('Loading safety fence');
% LoadObject("safetyfence.ply",[xScenePosition,yScenePosition,heightAdjustment],sceneOrientation+0);

bricks = cell(size(brickPositions));

% Load in all the bricks in their se positions
disp('Loading bricks');
for i=1:1:size(brickPositions)
   bricks{i} = LoadObject("Brick.ply",[brickPositions(i,1),brickPositions(i,2),brickPositions(i,3)],sceneOrientation+pi/2); 
end

% Set robot base positions so they are in the same spot on the table regardless of scene position or orientation
ur3.model.base = ur3.model.base * transl(0.65*cos(sceneOrientation)+xScenePosition,0.75*sin(sceneOrientation)+yScenePosition,tableSurfaceHeight)*trotz(sceneOrientation);
ur5.model.base = ur5.model.base* transl(0*sin(sceneOrientation)+yScenePosition,tableSurfaceHeight,0*cos(sceneOrientation)+xScenePosition)*troty(sceneOrientation); % y, z, x

end

