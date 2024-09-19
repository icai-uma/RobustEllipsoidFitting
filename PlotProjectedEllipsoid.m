% Plot the projection of a 3D ellipsoid on a 2D bitmap, given its center, its distance matrix, the
% desired plot color, the number of lines to be plotted, the stereo camera
% parameters, and the index (1 or 2) of the camera which captured the
% bitmap
function [Result] = PlotProjectedEllipsoid(Center,DistanceMatrix,MyColor,NumPoints,StereoCameraParameters,NdxCamera)


if ~isempty(find(DistanceMatrix-DistanceMatrix == 0, 1))
    
    if size(Center,1) < size(Center,2), Center = Center'; end
    
    
    theta = (0:1:NumPoints-1)'/(NumPoints-1)*pi;
    phi = (0:1:NumPoints-1)/(NumPoints-1)*2*pi;
    
    sx = sin(theta)*cos(phi);
    sy = sin(theta)*sin(phi);
    sz = cos(theta)*ones(1,NumPoints);
    
    svect = [reshape(sx,1,NumPoints*NumPoints); reshape(sy,1,NumPoints*NumPoints); reshape(sz,1,NumPoints*NumPoints)];
    epoints = sqrtm(DistanceMatrix) * svect  + Center*ones(1,NumPoints*NumPoints);

    % Backproject the points from 3D to the coordinate system of the first
    % camera (NdxCamera==1) or the second camera (NdxCamera==2)
    ex=zeros(NumPoints,NumPoints);
    ey=zeros(NumPoints,NumPoints);
    for NdxPoint=1:NumPoints*NumPoints
        reprojectedPoints = backprojectOnePoint(StereoCameraParameters, epoints(:,NdxPoint));
        ex(NdxPoint)=reprojectedPoints(NdxCamera,1);
        ey(NdxPoint)=reprojectedPoints(NdxCamera,2);
    end

    
    
    % Draw horizontal connections
    for NdxRow=1:NumPoints
        for NdxCol=1:(NumPoints-1)
            LineX=[ex(NdxRow,NdxCol) ex(NdxRow,NdxCol+1)];
            LineY=[ey(NdxRow,NdxCol) ey(NdxRow,NdxCol+1)];
            plot(LineX,LineY,'-k','LineWidth',0.5,'Color',MyColor);
        end
    end
    
    % Draw vertical connections
    for NdxRow=1:(NumPoints-1)
        for NdxCol=1:NumPoints
            LineX=[ex(NdxRow,NdxCol) ex(NdxRow+1,NdxCol)];
            LineY=[ey(NdxRow,NdxCol) ey(NdxRow+1,NdxCol)];
            plot(LineX,LineY,'-k','LineWidth',0.5,'Color',MyColor);
        end
    end


%     ez = reshape(epoints(3,:),NumPoints,NumPoints);
%     
%     Handle = mesh(ex,ey,ez, reshape(ones(NumPoints*NumPoints,1)*MyColor,NumPoints,NumPoints,3) );
%     hidden off

    
    Result=1;

else
  fprintf('\nVery ill covariance matrix - not plotting this one\n')
  Result=0;
end


