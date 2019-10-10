clc
clear all;

% Register gptoolbox
addpath(genpath('external'));
addpath 'functions'
addpath '191010_registration'
addpath 'Non_rigid_ICP'

[V1, F1, FB1, Header1] = function_loading_ply_file('S01_hand data.ply'); % template / source
[V2, F2, FB2, Header2] = function_loading_ply_file('S02_hand data.ply'); % target / scan

V2(:,4:6) = [];


%% rigid registration by MPII human shape (rigidAlignTemplate2Scan)

LM1 = function_get_LM_from_iges('S01_landmarks.igs')
LM2 = function_get_LM_from_iges('S02_landmarks.igs')

[regParams,Bfit,ErrorStats] = absor(LM1',LM2','doScale',true)

T = eye(4);
T(1:3,1:3) = regParams.R/regParams.s;
pointsOrig = V2;

points = [V2 ones(size(V2,1),1)]*T;
V2 = points(:,1:3);
t = mean(V1) - mean(V2);
V2 = V2 + repmat(t,size(V2,1),1);

T(4,1:3) = t;
T = T';

landmarks = [LM2, ones(size(LM2,1),1)]*T';
LM2 = landmarks(:,1:3);

points = [pointsOrig ones(size(V2,1),1)]*T';
V2 = points(:,1:3);

B = V2;
A = V1; 

figure(3) % point cloud 3D plotting
hold on
axis equal
scatter3(A(:,1),A(:,2),A(:,3),'.', 'MarkerEdgeColor',[217/255, 217/255, 217/255])
scatter3(B(:,1),B(:,2),B(:,3),'.', 'MarkerEdgeColor',[255/255, 0/255, 0/255])
hold off

%% non-rigid ICP registration

flag = 1;
iterations = 10;
flag_prealligndata = 1;
figureOn = 1;

[sourceV] = nonrigidICP(V2, V1, F2, F1, iterations, flag_prealligndata)

AO = sourceV;

figure(2) % point cloud 3D plotting
hold on
axis equal
scatter3(AO(:,1),AO(:,2),AO(:,3),'.', 'MarkerEdgeColor',[217/255, 217/255, 217/255])
% scatter3(Sorted_V(1,1),Sorted_V(1,2),Sorted_V(1,3), '*', 'MarkerEdgeColor',[1, 0, 0])
hold off


