function SpatialV2Path = setSpatialV2Path(filename)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
answer = inputdlg('Enter the absolute path to spatial v2 extended','Spatial V2 Path');
SpatialV2Path = answer{1};
if nargin > 0
    save(filename,'SpatialV2Path');
else
    save('SpatialV2Path.mat','SpatialV2Path');
end
end