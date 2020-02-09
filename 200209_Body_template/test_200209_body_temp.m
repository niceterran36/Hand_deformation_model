addpath('F:\[GitHub]\Hand_deformation_model\functions');
addpath(genpath('external'));

% [V1, F1, FB1, Header1] = function_loading_ply_file('newTemplate_body_WS_200209.ply'); % Wonsup's template original
%V2 = V1+[0 145.758 867.945]; % vertex translation for modification

[V1, F1, FB1, Header1] = function_loading_ply_file('newTemplate_body_HY_200209.ply'); % modified template (vertex location)
%load('COR.mat'); % original template COR
load('COR_new.mat');

C = COR_new;
A = V1;

figure()
hold on
axis equal
scatter3(A(:,1),A(:,2),A(:,3),'.', 'MarkerEdgeColor',[180/255, 180/255, 180/255])
plot3(C(:,1),C(:,2),C(:,3),'b*')
plot3(C([1 10 11 12 13],1),C([1 10 11 12 13],2),C([1 10 11 12 13],3), 'k-');
plot3(C(1:5,1),C(1:5,2),C(1:5,3), 'k-');
plot3(C([1 6:9],1),C([1 6:9],2),C([1 6:9],3), 'k-');
plot3(C(14:17,1),C(14:17,2),C(14:17,3), 'k-');
plot3(C(18:21,1),C(18:21,2),C(18:21,3), 'k-');
plot3(C([14 11 18],1),C([14 11 18],2),C([14 11 18],3), 'k-');
hold off

