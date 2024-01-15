function libpath = getCasADiPath()
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
pathfile = "CasADiPath.mat";
if isfile(pathfile)
    data = load(pathfile,'CasADiPath');
    libpath = data.CasADiPath;
else
    libpath = setCasADiPath(pathfile);
end
end