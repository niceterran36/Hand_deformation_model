% register library - PC
addpath(genpath('../external'));
addpath('F:\[GitHub]\Hand_deformation_model\functions');
addpath('F:\[GitHub]\Hand_deformation_model\data');
addpath('F:\[GitHub]\Hand_deformation_model\external\registration');
format shortG

% detection template LM index using new data (hand_mesh_for_LM.ply,
% LMt_new.igs)

LMt = function_get_LM_from_iges('LMt_new.igs');
[V, F, FB, H] = function_loading_ply_file('hand_mesh_for_LM.ply');

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
clear delta distances F FB H i j LMt m V Template_LM

