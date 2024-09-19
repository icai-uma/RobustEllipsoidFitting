function [BestCenter,BestDistanceMatrix,Errors,Volumes,Centers,StepSizes,GradientVectors]=FitEllipsoidEnhancedOpt(Samples,Lambda,StepSize,MaxStepSize,MinStepSize,NumSteps)
% Fit an ellipsoid to data, enhanced version with MEX optimization (MMX
% library)
% Gradient descent with adaptive step size. The best solution is returned
% rather than the last one.
% Inputs:
%   Samples=Matrix of size Dimension x NumSamples with the training samples
% Outputs:
%   BestCenter=Vector of size Dimension x 1 with the center of the fitted
%       ellipsoid.
%   BestDistanceMatrix=Matrix of size Dimension x Dimension with the distance
%       matrix of the fitted ellipsoid.

[Dimension, NumSamples]=size(Samples);
addpath('./mmx_package');

% Initialize the center to the mean of the distribution and the distance
% matrix to the covariance matrix of the distribution
Center=mean(Samples,2);
DistanceMatrix=cov(Samples');

% Initialize log variables
Errors=zeros(1,NumSteps);
DetDistMat=zeros(1,NumSteps);
Centers=zeros(Dimension,NumSteps);
StepSizes=zeros(1,NumSteps);
StepSizes(1)=StepSize;
GradientVectors=zeros(Dimension,NumSteps);

% Main loop
BestError=inf;
for NdxStep=1:NumSteps
    % Compute the distances to the surface of the current ellipsoid
    DistMatInv=inv(DistanceMatrix);
    Differences=Samples-repmat(Center,[1 NumSamples]);
    %DistMatInvDiff=multiprod(DistMatInv,Differences,[1 2],1); % z=inv(B)*d
    DistMatInvDiff=mmx_naive('mult',DistMatInv,Differences);
    SquaredDistances=dot(Differences,DistMatInvDiff); % SquaredDistances=d*inv(B)*d
    Distances=sqrt(SquaredDistances);
    
    % Find the points inside and outside the ellipsoid
    InnerPoints=SquaredDistances<1;
    OuterPoints=~InnerPoints;
    NumInnerPoints=nnz(InnerPoints);
    NumOuterPoints=nnz(OuterPoints);

    % Find the gradient of the distance matrix
    
    %DiffDiffT=multiprod(Differences, Differences, [1 0], [0 1]);
    DiffDiffT=mmx_naive('mult',reshape(Differences,[Dimension 1 NumSamples]),reshape(Differences,[1 Dimension NumSamples]));
    %DistMatInvT_DiffDiffT=multiprod(DistMatInv',DiffDiffT);
    DistMatInvT_DiffDiffT=mmx_naive('mult',DistMatInv',DiffDiffT);
    %DistMatInvT_DiffDiffT_DistMatInvT=multiprod(DistMatInvT_DiffDiffT,DistMatInv');
    DistMatInvT_DiffDiffT_DistMatInvT=mmx_naive('mult',DistMatInvT_DiffDiffT,DistMatInv');
    %DistMatInv_Diff=multiprod(DistMatInv,Differences);
    DistMatInv_Diff=mmx_naive('mult',DistMatInv,Differences);
    DistMatGradient=...
        (sum(DistMatInvT_DiffDiffT_DistMatInvT(:,:,InnerPoints)./ ...
        repmat(reshape(Distances(InnerPoints),[1 1 NumInnerPoints]),[Dimension Dimension 1]),3) - ...
        sum(DistMatInvT_DiffDiffT_DistMatInvT(:,:,OuterPoints)./ ...
        repmat(reshape(Distances(OuterPoints),[1 1 NumOuterPoints]),[Dimension Dimension 1]),3))/NumSamples;
    
    % Update the distance matrix
    DistanceMatrix=DistanceMatrix-StepSize*DistMatGradient;
    
    % Find the gradient of the center vector
    CenterGradient=...
        (sum(DistMatInv_Diff(:,InnerPoints)./ ...
        repmat(Distances(InnerPoints),[Dimension 1]),2) - ...
        sum(DistMatInv_Diff(:,OuterPoints)./ ...
        repmat(Distances(OuterPoints),[Dimension 1]),2))/NumSamples + ...
        Lambda*Center/norm(Center);
    GradientVectors(:,NdxStep)=CenterGradient;
    
    % Update the center vector
    Center=Center-StepSize*CenterGradient;
    
    % Update the logs
    Errors(NdxStep)=(sum(Distances(OuterPoints)-1)+...
        sum(1-Distances(InnerPoints)))/NumSamples+Lambda*norm(Center);
    DetDistMat(NdxStep)=det(DistanceMatrix);
    Centers(:,NdxStep)=Center;
    
    % Update the best solution found so far
    if Errors(NdxStep)<BestError
        BestError=Errors(NdxStep);
        BestCenter=Center;
        BestDistanceMatrix=DistanceMatrix;
    end
    
    % Update the step size each 10 iterations of the main loop
    if mod(NdxStep,10)==0
        % Check whether the error has grown in a robust way
        if median(Errors((NdxStep-4):NdxStep))<median(Errors((NdxStep-9):(NdxStep-5)))
            % The error is smaller, so we increase the step size provided
            % that it is not too big
            if StepSize<MaxStepSize
                StepSize=1.1*StepSize;
            end
        else
            % The error is bigger, so we decrease the step size provided
            % that it is not too small
            if StepSize>MinStepSize
                StepSize=0.9*StepSize;
            end
        end
    end
    StepSizes(NdxStep)=StepSize;
end

% Compute the volume of the fitted ellipsoids
Volumes=(pi^(Dimension/2))*sqrt(DetDistMat)/gamma((Dimension/2)+1);

