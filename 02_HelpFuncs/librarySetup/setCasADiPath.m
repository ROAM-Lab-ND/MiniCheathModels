function CasADiPath = setCasADiPath(filename)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
answer = inputdlg('Enter the absolute path to CasADi','CasADi Path');
CasADiPath = answer{1};
if nargin > 0
    save(filename,'CasADiPath');
else
    save('CasADiPath.mat','CasADiPath');
end
end