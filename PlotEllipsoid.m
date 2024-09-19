% Plot an ellipsoid in 3D, given its center, its distance matrix, the
% desired plot color, and the number of lines to be plotted
function [Handle] = PlotEllipsoid(Center,DistanceMatrix,MyColor,NumPoints)

if ~isempty(find(DistanceMatrix-DistanceMatrix == 0, 1))

  if size(Center,1) < size(Center,2), Center = Center'; end

  if size(DistanceMatrix,1) == 3

    theta = (0:1:NumPoints-1)'/(NumPoints-1)*pi;
    phi = (0:1:NumPoints-1)/(NumPoints-1)*2*pi;
    
    sx = sin(theta)*cos(phi);
    sy = sin(theta)*sin(phi);
    sz = cos(theta)*ones(1,NumPoints);
    
    svect = [reshape(sx,1,NumPoints*NumPoints); reshape(sy,1,NumPoints*NumPoints); reshape(sz,1,NumPoints*NumPoints)];
    epoints = sqrtm(DistanceMatrix) * svect  + Center*ones(1,NumPoints*NumPoints);
    
    ex = reshape(epoints(1,:),NumPoints,NumPoints);
    ey = reshape(epoints(2,:),NumPoints,NumPoints);
    ez = reshape(epoints(3,:),NumPoints,NumPoints);
    
    Handle = mesh(ex,ey,ez, reshape(ones(NumPoints*NumPoints,1)*MyColor,NumPoints,NumPoints,3) );
    hidden off

    
  else
    
    theta = (0:1:NumPoints-1)/(NumPoints-1)*2*pi;
    
    epoints = sqrtm(DistanceMatrix) * [cos(theta); sin(theta)]*1   + Center*ones(1,NumPoints);
    

    hold on
    Handle = plot(epoints(1,:),epoints(2,:),'LineWidth',3,'Color',MyColor);
    plot(Center(1,:),Center(2,:),'.g','Color',MyColor,'LineWidth',3); 

    
  end

else
  fprintf('\nVery ill covariance matrix - not plotting this one\n')
end
