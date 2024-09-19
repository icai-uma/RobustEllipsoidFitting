clear all
close all

addpath('..');

disp('Please, select image to process:');
disp('   1. Basketball');
disp('   2. Loquat');
disp('   3. Red balloon');
selection = input('Choose one option: ');

switch selection
    case 1
        image_sufix = '0123';
        Factor = 500;
        xyzFilter = [-2500 2500; -2000 3000; 10000 15000];
        xyzLimView = [-2500 2500; -2000 3500; 10000 20000];
        viewParam = [-170 -80];
    case 2
        image_sufix = '0139';
        Factor = 100;
        xyzFilter = [-2000 -200; 1100 3000; 16000 18500];
        xyzLimView = [-4000 2000; 0 5000; 0 22000];
        viewParam = [159 -69];
    case 3
        image_sufix = '0141';
        Factor = 100;
        xyzFilter = [0 2000; -700 1500; 7000 10000];
        xyzLimView = [-1000 3000; -1000 2000; 6000 12000];
        viewParam = [13 -77];
    otherwise
        error('Inexitent option');
end


%% READ CALIBRATION PARAMETERS OF THE CAMERA AND THE PAIR OF IMAGES TO PROCESS
load('./StereoParameters.mat');
I1 = imread(['./StereoData/Left/LEFT_',image_sufix,'.jpg']);
I2 = imread(['./StereoData/Right/RIGHT_',image_sufix,'.jpg']);

%% DELETE LENS DISTORTIONS

I1 = undistortImage(I1,stereoParams.CameraParameters1);
I2 = undistortImage(I2,stereoParams.CameraParameters2);

%% AUTODETECT PAIRS OF POINTS IN THE LEFT AND RIGHT IMAGES
% Detect dense feature points
imagePoints1 = detectMinEigenFeatures(rgb2gray(I1), 'MinQuality', 0.001);

% Create the point tracker
tracker = vision.PointTracker('MaxBidirectionalError', 1, 'NumPyramidLevels', 5);

% Initialize the point tracker
imagePoints1 = imagePoints1.Location;
initialize(tracker, imagePoints1, I1);

% Track the points
[imagePoints2, validIdx] = step(tracker, I2);
matchedPoints1 = imagePoints1(validIdx, :);
matchedPoints2 = imagePoints2(validIdx, :);

%% TRIANGULATE PAIRS OF POINTS

% Compute the 3-D points
points3D = triangulate(matchedPoints1, matchedPoints2, stereoParams);

% Filter 3-D points which are too far from the coordinate origin
GoodPoints=(points3D(:,1)>xyzFilter(1,1)) & (points3D(:,1)<xyzFilter(1,2)) & ...
    (points3D(:,2)>xyzFilter(2,1)) & (points3D(:,2)<xyzFilter(2,2)) & ...
    (points3D(:,3)>xyzFilter(3,1)) & (points3D(:,3)<xyzFilter(3,2));

points3D=points3D(GoodPoints,:);
matchedPoints1=matchedPoints1(GoodPoints,:);
matchedPoints2=matchedPoints2(GoodPoints,:);

% Get the color of each reconstructed point
numPixels = size(I1, 1) * size(I1, 2);
allColors = reshape(I1, [numPixels, 3]);
colorIdx = sub2ind([size(I1, 1), size(I1, 2)], round(matchedPoints1(:,2)), ...
    round(matchedPoints1(:, 1)));
color = allColors(colorIdx, :);

% Create the point cloud
ptCloud = pointCloud(points3D, 'Color', color);

%% VISUALIZE THE RESULTS OF THE TRIANGULATION
% Visualize the camera locations and orientations
cameraSize = 0.3;
figure
plotCamera('Size', cameraSize, 'Color', 'r', 'Label', '1', 'Opacity', 0);
hold on
grid on
plotCamera('Location', stereoParams.TranslationOfCamera2, 'Orientation', stereoParams.RotationOfCamera2, 'Size', cameraSize, ...
    'Color', 'b', 'Label', '2', 'Opacity', 0);

% Visualize the point cloud
pcshow(ptCloud, 'VerticalAxis', 'y', 'VerticalAxisDir', 'down', ...
    'MarkerSize', 45);

% Take an ellipsoid and plot it in the 3D world
Samples=double(ptCloud.Location');
MeanSamples=mean(Samples,2);
TransfSamples=(Samples-repmat(MeanSamples,[1 size(Samples,2)]))/Factor;
NumSteps=20000;
Lambda=0.001;
StepSize=0.3;
MaxStepSize=2;
MinStepSize=0.001;
[Center,DistanceMatrix,Errors,Volumes,Centers,StepSizes,GradientVectors]=FitEllipsoidEnhancedOpt(TransfSamples,Lambda,StepSize,MaxStepSize,MinStepSize,NumSteps);
Center=Center*Factor+MeanSamples;
DistanceMatrix=DistanceMatrix*(Factor^2);

MyColor=[1 0 0];
NumPoints=10;
[Handle] = PlotEllipsoid(Center,DistanceMatrix,MyColor,NumPoints);

% Rotate and zoom the plot
camorbit(0, -30);
camzoom(1.5);

% Label the axes
xlabel('x-axis');
ylabel('y-axis');
zlabel('z-axis')

% View parameters
view(viewParam)
xlim(xyzLimView(1,:));
ylim(xyzLimView(2,:));
zlim(xyzLimView(3,:));

% title('Up to Scale Reconstruction of the Scene');

% Example of point triangulation
matchingPoints=zeros(2,2);
matchingPoints(1,:)=matchedPoints1(1,:);
matchingPoints(2,:)=matchedPoints2(1,:);
[point3d, reprojectedPoints, reprojectionErrors] = ...
    triangulateOnePoint(stereoParams, matchingPoints);

% Reprojection of the ellipsoid on the image from the first view
figure;
[Handle] =image(I1);
hold on
PlotProjectedEllipsoid(Center,DistanceMatrix,MyColor,NumPoints,stereoParams,1);
hold off

% Reprojection of the ellipsoid on the image from the second view
figure;
[Handle] =image(I2);
hold on
PlotProjectedEllipsoid(Center,DistanceMatrix,MyColor,NumPoints,stereoParams,2);
hold off
