% Triangulate a point from a match from the images of the two cameras of a
% stereo system.
% Inputs:
%   stereoParams=The stereo camera parameter object
%   matchingPoints=2x2 matrix with the pixel coordinates of the matching point in
%       the images, matchingPoints(1,:) are the pixel coordinates in the
%       image captured by the first camera, and matchingPoints(2,:) are the
%       pixel coordinates in the image captured by the second camera.
% Output:
%   point3d=3x1 vector with the point to be backprojected, expressed in the
%       coordinate system of the first camera
% reprojectedPoints=2x2 matrix with the reprojected pixel coordinates of the matching point in
%       the images, reprojectedPoints(1,:) are the pixel coordinates in the
%       image captured by the first camera, and reprojectedPoints(2,:) are the
%       pixel coordinates in the image captured by the second camera.
% reprojectionErrors=2x2 matrix with the reprojection errors with respect to the input matching
%       points

function [point3d, reprojectedPoints, reprojectionErrors] = ...
    triangulateOnePoint(stereoParams, matchingPoints)


cameraMatrix1 = cameraMatrix(stereoParams.CameraParameters1, eye(3), [0 0 0]);
cameraMatrix2 = cameraMatrix(stereoParams.CameraParameters2, ...
    stereoParams.RotationOfCamera2, stereoParams.TranslationOfCamera2);
cameraMatrices = cat(3, cameraMatrix1, cameraMatrix2);

% do the triangulation
numViews = size(cameraMatrices, 3);
A = zeros(numViews * 2, 4);
for i = 1:numViews
    P = cameraMatrices(:,:,i)';
    A(2*i - 1, :) = matchingPoints(i, 1) * P(3,:) - P(1,:);
    A(2*i    , :) = matchingPoints(i, 2) * P(3,:) - P(2,:);
end

[~,~,V] = svd(A);
X = V(:, end);
X = X/X(end);

point3d = X(1:3)';

reprojectedPoints = zeros(size(matchingPoints), 'like', matchingPoints);
for i = 1:numViews
    reprojectedPoints(i, :) = projectPoints(point3d, cameraMatrices(:, :, i));
end
reprojectionErrors = reprojectedPoints - matchingPoints;
end

function points2d = projectPoints(points3d, P)
points3dHomog = [points3d, ones(size(points3d, 1), 1, 'like', points3d)];
points2dHomog = points3dHomog * P;
points2d = bsxfun(@rdivide, points2dHomog(:, 1:2), points2dHomog(:, 3));
end
