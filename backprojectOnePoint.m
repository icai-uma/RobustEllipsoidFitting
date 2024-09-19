% Backproject a 3D point expressed in the coordinates of the first camera
% Inputs:
%   stereoParams=The stereo camera parameter object
%   point3D=3x1 vector with the point to be backprojected, expressed in the
%       coordinate system of the first camera
% Output:
%   reprojectedPoints=2x2 matrix with the pixel coordinates of the point in
%       the images, reprojectedPoints(1,:) are the pixel coordinates in the
%       image captured by the first camera, and reprojectedPoints(2,:) are the
%       pixel coordinates in the image captured by the second camera.
function [reprojectedPoints] = ...
    backprojectOnePoint(stereoParams, point3d)


cameraMatrix1 = cameraMatrix(stereoParams.CameraParameters1, eye(3), [0 0 0]);
cameraMatrix2 = cameraMatrix(stereoParams.CameraParameters2, ...
    stereoParams.RotationOfCamera2, stereoParams.TranslationOfCamera2);
cameraMatrices = cat(3, cameraMatrix1, cameraMatrix2);

numViews = size(cameraMatrices, 3);

reprojectedPoints = zeros(2,2);
point3d=point3d';
for i = 1:numViews
    reprojectedPoints(i, :) = projectPoints(point3d, cameraMatrices(:, :, i));
end

end

function points2d = projectPoints(points3d, P)
points3dHomog = [points3d, ones(size(points3d, 1), 1, 'like', points3d)];
points2dHomog = points3dHomog * P;
points2d = bsxfun(@rdivide, points2dHomog(:, 1:2), points2dHomog(:, 3));
end
