addpath(genpath('./'));
rmpath(genpath('./.git'));
addpath(genpath(getSpatialV2Path()));
rmpath(genpath([getSpatialV2Path(),'/.git']));
addpath(getCasADiPath(),'-end');