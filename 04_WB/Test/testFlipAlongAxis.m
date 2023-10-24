addpath(genpath('./'));
addpath(genpath("../../spatial_v2_extended"));

a = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10];
aref = [1, 2, -3, 4, 5, 6, 7, -8, 9, -10];

A = flipAlongAxis(SpatialInertiaFromMassProperty(a), 'y');
Aref = SpatialInertiaFromMassProperty(aref);

A- Aref