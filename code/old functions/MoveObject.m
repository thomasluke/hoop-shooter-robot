function[objectMesh_h] = MoveObject(objectMesh_h,position,orientation)
% MoveObject moves object to a specified position and orientation

v = objectMesh_h.Vertices;

% Get vertex count
objectVertexCount = size(v,1);

% Move center point to origin
midPoint = sum(v)/objectVertexCount;
objectVerts = v - repmat(midPoint,objectVertexCount,1);

brickHeight = 0.06671; % meters

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
objectMesh_h.Vertices = updatedPoints(:,1:3);
drawnow();

end