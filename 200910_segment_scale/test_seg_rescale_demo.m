addpath(genpath('../external'));
addpath('F:\[GitHub]\Hand_deformation_model\functions');
addpath('F:\[GitHub]\Hand_deformation_model\data');
addpath('F:\[GitHub]\Hand_deformation_model\external\registration');
format shortG

load('CP.mat');
load('V11.mat');
load('F11.mat');
V = V11;
F = F11;
clear V11 F11
C1 = CP(2,:);
C2 = CP(1,:);
target_length = 25;

[V_tr, jCOR1_tr, jCOR2_tr] = segment_scale(V,F,C1,C2,target_length);

figure()
axis equal
axis off
hold on
scatter3(V(:,1),V(:,2),V(:,3),'.', 'MarkerEdgeColor',[190/255, 120/255, 5/255]);
scatter3(C1(:,1),C1(:,2),C1(:,3),'o','MarkerEdgeColor',[255/255, 0/255, 0/255]);
scatter3(C2(:,1),C2(:,2),C2(:,3),'o','MarkerEdgeColor',[255/255, 0/255, 0/255]);
scatter3(jCOR2_tr(:,1),jCOR2_tr(:,2),jCOR2_tr(:,3),'o','MarkerEdgeColor',[0/255, 0/255, 255/255]);
hold off

figure()
axis equal
axis off
hold on
scatter3(V_tr(:,1),V_tr(:,2),V_tr(:,3),'.', 'MarkerEdgeColor',[190/255, 120/255, 5/255]);
scatter3(C1(:,1),C1(:,2),C1(:,3),'o','MarkerEdgeColor',[255/255, 0/255, 0/255]);
scatter3(C2(:,1),C2(:,2),C2(:,3),'o','MarkerEdgeColor',[255/255, 0/255, 0/255]);
scatter3(jCOR2_tr(:,1),jCOR2_tr(:,2),jCOR2_tr(:,3),'o','MarkerEdgeColor',[0/255, 0/255, 255/255]);
hold off
