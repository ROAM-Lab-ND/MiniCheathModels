function libpath = getSpatialV2Path()
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
pathfile = "SpatialV2Path.mat";
if isfile(pathfile)
    data = load(pathfile,'SpatialV2Path');
    libpath = data.SpatialV2Path;
else
    libpath = setSpatialV2Path(pathfile);
end
end