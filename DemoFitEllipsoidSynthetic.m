% Fit a randomly generated ellipsoid (ground truth available)
clear all
close all

%rng('default');
rng(2);
Dimension=3;
NumSamples=100;
NumSteps=20000;
Lambda=0.001;
StepSize=0.3;
MaxStepSize=2;
MinStepSize=0.001;
NumPlotPoints=30;

TrueCenter=randn(Dimension,1);

% Generate a random positive semi-definite matrix.
TrueDistanceMatrix=rand(Dimension,100);
TrueDistanceMatrix=TrueDistanceMatrix*TrueDistanceMatrix';
TrueTransformMatrix=chol(TrueDistanceMatrix)';


% Generate samples so that the true distance matrix is TrueDistanceMatrix
% and the true center is TrueCenter
% Obtain random samples evenly distributed on the surface of the unit hypersphere
Samples=randn(Dimension,NumSamples);
SampleNorms=sqrt(sum(Samples.^2,1));
Samples=Samples./repmat(SampleNorms,[Dimension 1]); 
% Add some noise
Samples=Samples+0.01*randn(size(Samples));
% Transform the data into the desired hyperellipsoid
Samples=TrueTransformMatrix*Samples+repmat(TrueCenter,[1 NumSamples]); 

% Check that the generated samples are on the desired hyperellipsoid
Differences=Samples-repmat(TrueCenter,[1 NumSamples]);
DistMatInvDiff=multiprod(inv(TrueDistanceMatrix),Differences,[1 2],1); % z=inv(B)*d
SquaredDistances=dot(Differences,DistMatInvDiff); % SquaredDistances=d*inv(B)*d
% Must be close to one
mean(SquaredDistances)

% Add outliers
Samples=[Samples 10*rand(3,10)];

% Fit the hyperellipsoid
tic
[FittedCenter,FittedDistanceMatrix,Errors,Volumes,Centers,LearningRates,GradientVectors]=FitEllipsoidEnhancedOpt(Samples,Lambda,StepSize,MaxStepSize,MinStepSize,NumSteps);
toc

% Plot the training samples, the true ellipsoid and the fitted ellipsoid
figure
plot3(Samples(1,:),Samples(2,:),Samples(3,:),'.g');
hold on
MyColor=[0 0 1];
PlotEllipsoid(TrueCenter,TrueDistanceMatrix,MyColor,NumPlotPoints);
MyColor=[1 0 0];
PlotEllipsoid(FittedCenter,FittedDistanceMatrix,MyColor,NumPlotPoints);
legend('Training samples','True ellipsoid','Fitted ellipsoid');

% Plot the errors
figure
plot(Errors,'-r');
xlabel('Step');
ylabel('Error');

% Plot the volumes
figure
plot(sqrt(sum(GradientVectors.^2,1)),'-b');
xlabel('Step');
ylabel('Norm of the gradient of the center vector');

% Plot the learning rates
figure
plot(LearningRates,'-r');
xlabel('Step');
ylabel('Learning rate');




