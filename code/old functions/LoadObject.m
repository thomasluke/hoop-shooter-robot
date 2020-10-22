function[objectMesh_h] = LoadObject(objectName, position, orientation)
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

% Create a transform to describe the location (at the origin, since it's centered
objectPose = eye(4);

innacuracyOffset = 0.006518;

% Move forwards (facing in -y direction)
forwardTR = makehgtform('translate',position+midPoint-innacuracyOffset);

% % Random rotate about Z
rotateTR = makehgtform('zrotate',orientation);

% Move the pose forward and a slight and random rotation
objectPose = objectPose * forwardTR*rotateTR;
updatedPoints = [objectPose * [objectVerts,ones(objectVertexCount,1)]']';  


% Now update the Vertices
objectMesh_h.Vertices = updatedPoints(:,1:3);
drawnow();

end