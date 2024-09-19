% Fit an ellipsoid to a set of 3D scanned data
clear all
close all

%rng('default');
%rng(2);
Dimension=3;
NumSamples=200;
NumSteps=20000;
Lambda=0.001;
StepSize=50;
MaxStepSize=100;
MinStepSize=0.001;
NumPlotPoints=30;

load('ScannerData/egg_2mm_02mm_spacing.mat','Samples');
%load('ScannerData/kiwi1_1_2mm_spacing.mat','Samples');
%load('ScannerData/kiwi2_2mm_spacing.mat','Samples');
%load('ScannerData/mandarin1_spacing.mat','Samples');
%load('ScannerData/apple_2mm.mat','Samples');
%load('ScannerData/grape_02mm_spacing.mat','Samples');
%load('ScannerData/grape2_02mm_spacing.mat','Samples');
%load('ScannerData2/egg_outlier.mat','Samples');
%load('ScannerData2/egg_outlier1.mat','Samples');
%load('ScannerData2/lightbulb.mat','Samples');
%load('ScannerData2/lightbulb_no_faces.mat','Samples');
%load('ScannerData2/lightbulb_no_faces.mat','Samples');
%load('ScannerData2/lightbulb_world.mat','Samples');
%load('ScannerData2/lightbulb_world1.mat','Samples');
%load('ScannerData2/world.mat','Samples');
%load('ScannerData3/kiwi_outliers.mat','Samples');
%load('ScannerData3/mandarin2mm.mat','Samples');
%load('ScannerData3/mandarin2mm_outliers.mat','Samples');

NumRawSamples=size(Samples,1);
MySampleIndices=randperm(NumRawSamples,NumSamples);

SmallSamples=Samples(MySampleIndices,:)';
% Fit the hyperellipsoid
[FittedCenter,FittedDistanceMatrix,Errors,Volumes,Centers,LearningRates,GradientVectors]=FitEllipsoidEnhancedOpt(SmallSamples,Lambda,StepSize,MaxStepSize,MinStepSize,NumSteps);

% Plot the training samples, the true ellipsoid and the fitted ellipsoid
figure
plot3(SmallSamples(1,:),SmallSamples(2,:),SmallSamples(3,:),'.g');
hold on
MyColor=[1 0 0];
PlotEllipsoid(FittedCenter,FittedDistanceMatrix,MyColor,NumPlotPoints);
legend('Training samples','Fitted ellipsoid');

% Plot the errors
figure
plot(Errors,'-r');
xlabel('Step');
ylabel('Error');

% Plot the learning rates
figure
plot(LearningRates,'-r');
xlabel('Step');
ylabel('Learning rate');


% Plot the volumes
figure
plot(Volumes,'-m');
xlabel('Step');
ylabel('Volume of the fitted ellipsoid');

% Plot the volumes
figure
plot(sqrt(sum(GradientVectors.^2,1)),'-b');
xlabel('Step');
ylabel('Norm of the gradient of the center vector');

% Plot the centers
figure
plot3(SmallSamples(1,:),SmallSamples(2,:),SmallSamples(3,:),'.g');
hold on
plot3(Centers(1,:),Centers(2,:),Centers(3,:),'.r');
hold on
title('Evolution of the center vector');

