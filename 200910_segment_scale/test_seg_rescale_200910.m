%% register library - PC
addpath(genpath('../external'));
addpath('F:\[GitHub]\Hand_deformation_model\functions');
addpath('F:\[GitHub]\Hand_deformation_model\data');
addpath('F:\[GitHub]\Hand_deformation_model\external\registration');
format shortG

%% fuction development

% input = V, F, jCOR1(fixed), jCOR2(moved), scale
% output = V, jCOR1, jCOR2

% [V_tr, jCOR1_tr, jCOR2_tr] = function(V,F,jCOR1,jCOR2,target_length)
% 
% LM1 = [jCOR1; jCOR2];
% C1 = jCOR1; C2 = jCOR2;
% vector = C2-C1;
% scale_factor = target_length/norm(vector);
% vector = scale_factor*vector;
% C2_n = C1+vector;
% vector2 = C2_n - C1;
% % fprintf()
% norm(vector2)
% 
% LM2 = [C1; C2_n];
% jCOR1_tr = C1;
% jCOR2_tr = C2_n;
% 
% [b,bc] = bbw_boundary_conditions(V,F,LM1);
% W = bbw_biharmonic_bounded(V,F,b,bc);
% W = W./repmat(sum(W,2),1,size(W,2));
% 
% Diff = LM2 - LM1;
% V_tr = bbw_simple_deform(V,F,LM1,W,Diff);


%%
load('CP.mat');
load('V9.mat');
load('V10.mat');
V11_t = load('V11.mat');

load('hy_mesh_n3.mat');
vertices = mesh.vertices;
faces = mesh.faces;

keep = ismember(mesh.assignment, 11);
[V11, F11] = filter_vertices(vertices, faces, keep);
LM1 = CP(1:2,:)

V11 = V11_t.V11

C1 = CP(1,:);
C2 = CP(2,:);
Lv = C1-C2;
finger_seg_lngth = 25;
scale_factor = finger_seg_lngth/norm(Lv);
Lv = scale_factor*Lv

C1_m = C2+Lv

Lv2 = C1_m - C2
norm(Lv2)

LM2 = LM1;
LM2(1,:) = C1_m;

figure()
axis equal
axis off
hold on
%scatter3(V9(:,1),V9(:,2),V9(:,3),'.', 'MarkerEdgeColor',[180/255, 180/255, 180/255]);
%scatter3(V10(:,1),V10(:,2),V10(:,3),'.', 'MarkerEdgeColor',[190/255, 240/255, 251/255]);
scatter3(V11(:,1),V11(:,2),V11(:,3),'.', 'MarkerEdgeColor',[190/255, 120/255, 5/255]);
%scatter3(CP(:,1),CP(:,2),CP(:,3),'o','MarkerEdgeColor',[255/255, 0/255, 0/255]);
scatter3(LM1(:,1),LM1(:,2),LM1(:,3),'o','MarkerEdgeColor',[255/255, 0/255, 0/255]);
scatter3(C1_m(:,1),C1_m(:,2),C1_m(:,3),'o','MarkerEdgeColor',[0/255, 0/255, 255/255]);
hold off
%%

[b,bc] = bbw_boundary_conditions(V11,F11,LM1);
W = bbw_biharmonic_bounded(V11,F11,b,bc);
W = W./repmat(sum(W,2),1,size(W,2));

Diff = LM2 - LM1;
V_morphed = bbw_simple_deform(V11,F11,LM1,W,Diff);

figure()
axis equal
axis off
hold on
scatter3(V_morphed(:,1),V_morphed(:,2),V_morphed(:,3),'.', 'MarkerEdgeColor',[190/255, 120/255, 5/255]);
%scatter3(CP(:,1),CP(:,2),CP(:,3),'o','MarkerEdgeColor',[255/255, 0/255, 0/255]);
scatter3(LM1(:,1),LM1(:,2),LM1(:,3),'o','MarkerEdgeColor',[255/255, 0/255, 0/255]);
scatter3(C1_m(:,1),C1_m(:,2),C1_m(:,3),'o','MarkerEdgeColor',[0/255, 0/255, 255/255]);
hold off








