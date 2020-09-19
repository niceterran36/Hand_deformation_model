% register library - PC
addpath(genpath('../external'));
addpath('F:\[GitHub]\Hand_deformation_model\functions');
addpath('F:\[GitHub]\Hand_deformation_model\data');
addpath('F:\[GitHub]\Hand_deformation_model\external\registration');
format shortG

% detection template LM index using new data (hand_mesh_for_LM.ply,
% LMt_new.igs)

LMt = function_get_LM_from_iges('LMt_new.igs');
LMs = function_get_LM_from_iges('LMs4_n.igs');
[V, F, FB, H] = function_loading_ply_file('hand_mesh_for_LM.ply');
[Vs, Fs, FBs, Hs] = function_loading_ply_file('hand_mesh_for_LM.ply');

m = size(V, 1);
Template_LM = zeros(24,1);

for i=1:24
delta = V - repmat(LMt(i, :), m, 1);
distances = sum(delta .^ 2, 2);
[~, j] = min(distances);
Template_LM(i,:) = j;
end
for i = 1:24
LMt(i,:) = V(Template_LM(i),:);
end 

idx_LMt_24 = Template_LM;
clear delta distances i j m Template_LM

Tcor = zeros(12,3);
Scor = zeros(12,3);
IDX = [1:3, 7:9, 13:15, 19:21];

for xx = 1:12
    Tcor(xx,:) = (LMt(IDX(xx),:) + LMt(IDX(xx)+3,:))/2;
    Scor(xx,:) = (LMs(IDX(xx),:) + LMs(IDX(xx)+3,:))/2;
end 


[b,bc] = bbw_boundary_conditions(V,F,Tcor);
W = bbw_biharmonic_bounded(V,F,b,bc);
W = W./repmat(sum(W,2),1,size(W,2));

P = zeros(size(Tcor,1),1)
P = [1:size(Tcor,1)]'
BE = [1 2; 2 3; 4 5; 5 6; 7 8; 8 9; 10 11; 11 12];
Tcor2 = Tcor;
Tcor2([2 3 5 6 8 9 11 12],:) = Tcor([2 3 5 6 8 9 11 12],:)+50;

[varargout] = bbw_skinning_transformations(Tcor,P,BE,Tcor2)

Diff = Scor - Tcor;
V_tr = bbw_simple_deform(V,F,LMt,W,Diff);

figure()
axis equal
axis off
hold on
scatter3(V_morphed(:,1),V_morphed(:,2),V_morphed(:,3),'.', 'MarkerEdgeColor',[190/255, 120/255, 5/255]);
%scatter3(CP(:,1),CP(:,2),CP(:,3),'o','MarkerEdgeColor',[255/255, 0/255, 0/255]);
scatter3(LM1(:,1),LM1(:,2),LM1(:,3),'o','MarkerEdgeColor',[255/255, 0/255, 0/255]);
scatter3(C1_m(:,1),C1_m(:,2),C1_m(:,3),'o','MarkerEdgeColor',[0/255, 0/255, 255/255]);
hold off










