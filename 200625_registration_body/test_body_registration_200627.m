%% register library - PC
addpath(genpath('../external'));
addpath('F:\[GitHub]\Hand_deformation_model\functions');
addpath('F:\[GitHub]\Hand_deformation_model\data');
addpath('F:\[GitHub]\Hand_deformation_model\external\registration');
format shortG
%% register library - Labtop
clc
clear all
addpath(genpath('../external'));
addpath('/Users/hayoungjung/Documents/GitHub/Hand_deformation_model/data');
addpath('/Users/hayoungjung/Documents/GitHub/Hand_deformation_model/functions');
format shortG
%% Load data - Initial
load('Body_temp.mat');
Body_template = Body_temp;
points = {}; [points.V, points.F, points.FB, points.H] = function_loading_ply_file('sample_body.ply'); 
points.V(:,4:6) = []; points.normals = per_vertex_normals(points.V, points.F);
clear Body_temp
%% Load Landmarks to initial registration
LMs = function_get_LM_from_iges('sample_body_LM.igs');
LMt = function_get_LM_from_iges('template_body_LM.igs');
%%
AF = Body_template.F; AV = Body_template.V; AN = Body_template.normals;
BF = points.F; BV = points.V; BN = points.normals;
figure()
hold on;
trimesh(AF, AV(:, 1), AV(:, 2), AV(:, 3), 'EdgeColor', 'none', 'FaceColor', [0.4, 0.9, 0.4], 'FaceAlpha', 0.5);
trimesh(BF, BV(:, 1), BV(:, 2), BV(:, 3), 'EdgeColor', 'none', 'FaceColor', [0.8, 0.8, 0.8], 'FaceAlpha', 0.5);
%quiver3(AV(:, 1), AV(:, 2), AV(:, 3), AN(:, 1), AN(:, 2), AN(:, 3), 'Color', [0.4, 0.9, 0.4]);
%quiver3(BV(:, 1), BV(:, 2), BV(:, 3), BN(:, 1), BN(:, 2), BN(:, 3), 'Color', [0.8, 0.8, 0.8]);
hold off;
view([-90, 0]); camlight; view([90, 0]); camlight; view([43,25]);
axis equal;
grid off;
lighting gouraud;
axis off;
title('Beginning State');
clear AF AV AN BF BV BN;

%% 
% size of template vertices = n, % size of scan points = m
n = size(Body_template.V, 1); 
m = size(points.V, 1);
Template_LM = zeros(size(LMs,1),1);

for i=1:size(LMs,1)
delta = Body_template.V - repmat(LMt(i, :), n, 1);
distances = sum(delta .^ 2, 2);
[~, j] = min(distances);
Template_LM(i,:) = j;
end
% template index based LMt info. update
for i = 1:size(LMs,1)
LMt(i,:) = Body_template.V(Template_LM(i),:);
end 
clear Body_temp i j delta distances m n;

%% Size adjustment by ABSOR
% torso adjustment, LM set for torso fitting (LM located around torso boundary

LMt_tor = LMt([6,7,10,11,13,41,42],:);
LMs_tor = LMs([6,7,10,11,13,41,42],:);
LMt_tor = LMt_tor'; LMs_tor = LMs_tor';
[regParams_tor,~,~] = absor(LMt_tor,LMs_tor);
Body_template.V = apply_matrix(regParams_tor.M, Body_template.V, 1);
% Body_template.COR = apply_matrix(regParams_tor.M, Body_template.COR, 1);
for i = 1:21
Body_template.spheres{1, i}.center = apply_matrix(regParams_tor.M, Body_template.spheres{1, i}.center, 1);
end
clear LMs_tor LMt_tor regParams_tor;

%%
% generation transformation matrix
transforms = cell(1, 21);
for i = 1 : 21
    transforms{i} = eye(4);
end
axes = bone_axes_body(Body_template.spheres);

%%
AF = Body_template.F; AV = Body_template.V; AN = Body_template.normals;
BF = points.F; BV = points.V; BN = points.normals;
figure()
hold on;
trimesh(AF, AV(:, 1), AV(:, 2), AV(:, 3), 'EdgeColor', 'none', 'FaceColor', [0.4, 0.9, 0.4], 'FaceAlpha', 0.5);
trimesh(BF, BV(:, 1), BV(:, 2), BV(:, 3), 'EdgeColor', 'none', 'FaceColor', [0.8, 0.8, 0.8], 'FaceAlpha', 0.5);
%quiver3(AV(:, 1), AV(:, 2), AV(:, 3), AN(:, 1), AN(:, 2), AN(:, 3), 'Color', [0.4, 0.9, 0.4]);
%quiver3(BV(:, 1), BV(:, 2), BV(:, 3), BN(:, 1), BN(:, 2), BN(:, 3), 'Color', [0.8, 0.8, 0.8]);
hold off;
view([0, 90]);camlight;view([90, 0]);camlight;view([0, 90]);
axis equal;grid off;lighting gouraud;axis off;title('Initial guess');
clear AF AV AN BF BV BN;
% pause;

%%
figure()
hold on;
axis equal
axis off
h = trimesh(Body_template.F, Body_template.V(:, 1), Body_template.V(:, 2), Body_template.V(:, 3), 'EdgeColor', 'none', 'FaceColor', [0.5, 0.5, 0.5], 'FaceAlpha', 1);
lighting gouraud;
view([-90, 0]);
camlight;
view([90, 0]);
camlight;
view([0, 90]);
hold off;

%%
% left upper arm adjustment
% LM set for left upper arm fitting

% template index based LMt info. update
LM_Luparm_idx = [2,4,5,6,20,21,22,24];
LMs_Luparm = LMs([2,4,5,6,20,21,22,24],:);

for i = 1:size(LMs_Luparm,1)
LMt_Luparm(i,:) = Body_template.V(Template_LM(LM_Luparm_idx(i)),:);
end 

LMt_Luparm = LMt_Luparm'; LMs_Luparm = LMs_Luparm';
[regParams_Luparm,~,~] = absor(LMt_Luparm,LMs_Luparm);

FRP_segment = [6 9 12 15 5];
FRP_cor = [2 6 14 18 12];
FRP_digits{1} = [6:8];
FRP_digits{2} = [9:11];
FRP_digits{3} = [12:14];
FRP_digits{4} = [15:17];
FRP_digits{5} = [5];

FRP_cor_tr{1} = [3:5];
FRP_cor_tr{2} = [7:9];
FRP_cor_tr{3} = [15:17];
FRP_cor_tr{4} = [19:21];
FRP_cor_tr{5} = [13];

transform_order= [2 6 14 18 12];

transform = eye(4);

delta = regParams_Luparm.M;
delta = constraint_transformation(delta, centers_c(FRP_cor(3),:));
transform = delta * transform;
vertices = apply_matrix(delta, vertices);
normals = apply_matrix(delta, normals, 0);
transforms{14} = transform;

%% Apply transformation
% left hip
transforms{2} = transforms{2};
% left knee
transforms{3} = transforms{3} * transforms{2};
% left ankle
transforms{4} = transforms{4} * transforms{3};
% right hip
transforms{6} = transforms{6};
% right knee
transforms{7} = transforms{7} * transforms{6};
% right ankle
transforms{8} = transforms{8} * transforms{7};
% left shoulder
transforms{14} = transforms{14};
% left elbow
transforms{15} = transforms{15} * transforms{14};
% left wrist
transforms{16} = transforms{16} * transforms{15};
% right shoulder
transforms{18} = transforms{18};
% right elbow
transforms{19} = transforms{19} * transforms{18};
% right wrist
transforms{20} = transforms{20}  * transforms{19};
% Neck
transforms{12} = transforms{12};

Body_template = skin_dualquat_body(Body_template, transforms);
for i = 1:21
     Body_template.COR(i,:) = Body_template.spheres{1,i}.center;
end

vertices_c = Body_template.V;
normals = per_vertex_normals(Body_template.V, Body_template.F);

figure()
hold on;
axis equal
axis off
h = trimesh(Body_template.F, Body_template.V(:, 1), Body_template.V(:, 2), Body_template.V(:, 3), 'EdgeColor', 'none', 'FaceColor', [0.5, 0.5, 0.5], 'FaceAlpha', 1);
lighting gouraud;
view([-90, 0]);
camlight;
view([90, 0]);
camlight;
view([0, 90]);
hold off;

figure()
axis equal
axis off
hold on
scatter3(points.vertices(:,1),points.vertices(:,2),points.vertices(:,3),'.', 'MarkerEdgeColor',[180/255, 180/255, 180/255]);
scatter3(Body_template.V(:,1),Body_template.V(:,2),Body_template.V(:,3),'.', 'MarkerEdgeColor',[190/255, 240/255, 251/255]);
% scatter3(hand_template.centers(:,1),hand_template.centers(:,2),hand_template.centers(:,3),'o','MarkerEdgeColor',[255/255, 0/255, 0/255]);
hold off

%% call left arm only
% Larm_idx = [12, 13, 14];
% 
% keep = ismember(Body_temp.v_segment, Larm_idx);
% vi_Larm = vertices(keep,:);
% 
% transforms = regParams_Luparm.M;
% 
% Body_template = skin_dualquat_body(Body_template, transforms);
% for i = 1:21
%      Body_template.COR(i,:) = Body_template.spheres{1,i}.center;
% end



vertices = apply_matrix(regParams_Luparm.M, vertices, 1);
centers_c = apply_matrix(regParams_Luparm.M, centers_c, 1);

LMt = LMt'; LMs = LMs';
clear j delta distances 

% apply transformation matrix
[regParams,Bfit,ErrorStats] = absor(LMt,LMs);
vertices = apply_matrix(regParams.M, vertices, 1);
centers_c = apply_matrix(regParams.M, centers_c, 1);



%% parameter for finger root (MCP) registration 

FRP_segment = [6 9 12 15 5];
FRP_cor = [2 6 14 18 12];
FRP_digits{1} = [6:8];
FRP_digits{2} = [9:11];
FRP_digits{3} = [12:14];
FRP_digits{4} = [15:17];
FRP_digits{5} = [5];

FRP_cor_tr{1} = [3:5];
FRP_cor_tr{2} = [7:9];
FRP_cor_tr{3} = [15:17];
FRP_cor_tr{4} = [19:21];
FRP_cor_tr{5} = [13];

transform_order= [2 6 14 18 12];

%% D1-D5 finger root (MCP) registration
h3 = [];
h4 = [];

for j = 1:5

vertices = vertices_c;
faces = faces_c;
normals = per_vertex_normals(vertices, faces);

keep = ismember(Body_temp.v_segment, FRP_segment(j));
[vertices, faces] = filter_vertices(vertices, faces, keep);
normals = normals(keep, :);
pairs = compute_correspondences_body2(vertices, normals, points.vertices, points.normals);

transform = eye(4);

figure(2)
    for i = 1 : 10
        delete(h3);
        delete(h4);
        delta = compute_transformation(vertices, points.vertices, points.normals, pairs);
        delta = constraint_transformation(delta, centers_c(FRP_cor(j),:));
        transform = delta * transform;
        vertices = apply_matrix(delta, vertices);
        normals = apply_matrix(delta, normals, 0);
        pairs = compute_correspondences_body(vertices, normals, points.vertices, points.normals);
        v = get(gca, 'view');
        trimesh(faces, vertices(:, 1), vertices(:, 2), vertices(:, 3), 'EdgeColor', 'none', 'FaceColor', [0.4, 0.9, 0.4], 'FaceAlpha', 0.1);
        hold on;
        h3 = trimesh(points.faces, points.vertices(:, 1), points.vertices(:, 2), points.vertices(:, 3), 'EdgeColor', 'none', 'FaceColor', [0.8, 0.8, 0.8], 'FaceAlpha', 0.1);
        h4 = plot3( ...
            [vertices(pairs(:, 1), 1), points.vertices(pairs(:, 2), 1)]', ...
            [vertices(pairs(:, 1), 2), points.vertices(pairs(:, 2), 2)]', ...
            [vertices(pairs(:, 1), 3), points.vertices(pairs(:, 2), 3)]', ...
        'Color', 'red');
        hold off;
        view([-90, 0]);
        camlight;
        view([90, 0]);
        camlight;
        axis equal;
        grid off;
        lighting gouraud;
        axis off;
        title(['After ', num2str(i), ' rigid transformation']);
        set(gca, 'view', v);
        pause(0.01);
    end

transforms{transform_order(j)} = transform;
   
keep = ismember(Body_temp.v_segment, FRP_digits{j});
vi_Dx = vertices_c(keep,:);
vi_Dx = apply_matrix(transform, vi_Dx, 1);

vertices_c(keep,:) = vi_Dx;
centers_c(FRP_cor_tr{j},:) = apply_matrix(transform, centers_c(FRP_cor_tr{j},:), 1);
normals = per_vertex_normals(vertices_c, faces);

end

figure(3)
axis equal
axis off
hold on
scatter3(points.vertices(:,1),points.vertices(:,2),points.vertices(:,3),'.', 'MarkerEdgeColor',[180/255, 180/255, 180/255]);
scatter3(vertices_c(:,1),vertices_c(:,2),vertices_c(:,3),'.', 'MarkerEdgeColor',[190/255, 240/255, 251/255]);
scatter3(centers_c(:,1),centers_c(:,2),centers_c(:,3),'o','MarkerEdgeColor',[255/255, 0/255, 0/255]);
hold off

%% Apply transformation
% left hip
transforms{2} = transforms{2};
% left knee
transforms{3} = transforms{3} * transforms{2};
% left ankle
transforms{4} = transforms{4} * transforms{3};
% right hip
transforms{6} = transforms{6};
% right knee
transforms{7} = transforms{7} * transforms{6};
% right ankle
transforms{8} = transforms{8} * transforms{7};
% left shoulder
transforms{14} = transforms{14};
% left elbow
transforms{15} = transforms{15} * transforms{14};
% left wrist
transforms{16} = transforms{16} * transforms{15};
% right shoulder
transforms{18} = transforms{18};
% right elbow
transforms{19} = transforms{19} * transforms{18};
% right wrist
transforms{20} = transforms{20}  * transforms{19};
% Neck
transforms{12} = transforms{12};

%% DQS application & Result display

Body_template = skin_dualquat_body(Body_template, transforms);
for i = 1:21
     Body_template.COR(i,:) = Body_template.spheres{1,i}.center;
end

figure()
hold on;
axis equal
axis off
h = trimesh(Body_template.F, Body_template.V(:, 1), Body_template.V(:, 2), Body_template.V(:, 3), 'EdgeColor', 'none', 'FaceColor', [0.5, 0.5, 0.5], 'FaceAlpha', 1);
lighting gouraud;
view([-90, 0]);
camlight;
view([90, 0]);
camlight;
hold off;

figure()
axis equal
axis off
hold on
scatter3(points.vertices(:,1),points.vertices(:,2),points.vertices(:,3),'.', 'MarkerEdgeColor',[180/255, 180/255, 180/255]);
scatter3(Body_template.V(:,1),Body_template.V(:,2),Body_template.V(:,3),'.', 'MarkerEdgeColor',[190/255, 240/255, 251/255]);
% scatter3(hand_template.centers(:,1),hand_template.centers(:,2),hand_template.centers(:,3),'o','MarkerEdgeColor',[255/255, 0/255, 0/255]);
hold off

vertices_c = Body_template.V;
normals = Body_template.normals;

%% parameter for fingers registration order

P_segment = [7 8 10 11 13 14 16 17];
P_cor = [3 4 7 8 15 16 19 20];

P_digits{1} = [7:8];
P_digits{2} = 8;
P_digits{3} = [10:11];
P_digits{4} = 11;
P_digits{5} = [13:14];
P_digits{6} = 14;
P_digits{7} = [16:17];
P_digits{8} = 17;

P_cor_tr{1} = [4:5];
P_cor_tr{2} = 5;
P_cor_tr{3} = [8:9];
P_cor_tr{4} = 9;
P_cor_tr{5} = [16:17];
P_cor_tr{6} = 17;
P_cor_tr{7} = [20:21];
P_cor_tr{8} = 21;

transform_order= [3 4 7 8 15 16 19 20];

%% leg, arm registration
h3 = [];
h4 = [];

transforms2 = cell(1, 21);
for i = 1 : 21
    transforms2{i} = eye(4);
end

for j = 1:8

vertices = vertices_c;
faces = faces_c;
normals = per_vertex_normals(vertices, faces);

keep = ismember(Body_temp.v_segment, P_segment(j));
[vertices, faces] = filter_vertices(vertices, faces, keep);
normals = normals(keep, :);
pairs = compute_correspondences_body2(vertices, normals, points.vertices, points.normals);

transform = eye(4);

figure(2)
    for i = 1 : 10
        delete(h3);
        delete(h4);
        delta = compute_transformation(vertices, points.vertices, points.normals, pairs);
        delta = constraint_transformation(delta, centers_c(P_cor(j),:));
        transform = delta * transform;
        vertices = apply_matrix(delta, vertices);
        normals = apply_matrix(delta, normals, 0);
        pairs = compute_correspondences_body(vertices, normals, points.vertices, points.normals);
        v = get(gca, 'view');
        trimesh(faces, vertices(:, 1), vertices(:, 2), vertices(:, 3), 'EdgeColor', 'none', 'FaceColor', [0.4, 0.9, 0.4], 'FaceAlpha', 0.1);
        hold on;
        h3 = trimesh(points.faces, points.vertices(:, 1), points.vertices(:, 2), points.vertices(:, 3), 'EdgeColor', 'none', 'FaceColor', [0.8, 0.8, 0.8], 'FaceAlpha', 0.1);
        h4 = plot3( ...
            [vertices(pairs(:, 1), 1), points.vertices(pairs(:, 2), 1)]', ...
            [vertices(pairs(:, 1), 2), points.vertices(pairs(:, 2), 2)]', ...
            [vertices(pairs(:, 1), 3), points.vertices(pairs(:, 2), 3)]', ...
        'Color', 'red');
        hold off;
        view([-90, 0]);
        camlight;
        view([90, 0]);
        camlight;
        axis equal;
        grid off;
        lighting gouraud;
        axis off;
        title(['After ', num2str(i), ' rigid transformation']);
        set(gca, 'view', v);
        pause(0.01);
    end

transforms2{transform_order(j)} = transform;
   
keep = ismember(Body_temp.v_segment, P_digits{j});
vi_Dx = vertices_c(keep,:);
vi_Dx = apply_matrix(transform, vi_Dx, 1);

vertices_c(keep,:) = vi_Dx;
centers_c(P_cor_tr{j},:) = apply_matrix(transform, centers_c(P_cor_tr{j},:), 1);
normals = per_vertex_normals(vertices_c, faces);

end

%% Apply transformation
% left hip
transforms{2} = transforms{2};
% left knee
transforms{3} = transforms{3} * transforms{2};
% left ankle
transforms{4} = transforms{4} * transforms{3};
% right hip
transforms{6} = transforms{6};
% right knee
transforms{7} = transforms{7} * transforms{6};
% right ankle
transforms{8} = transforms{8} * transforms{7};
% left shoulder
transforms{14} = transforms{14};
% left elbow
transforms{15} = transforms{15} * transforms{14};
% left wrist
transforms{16} = transforms{16} * transforms{15};
% right shoulder
transforms{18} = transforms{18};
% right elbow
transforms{19} = transforms{19} * transforms{18};
% right wrist
transforms{20} = transforms{20}  * transforms{19};
% Neck
transforms{12} = transforms{12};

%% DQS application & Result display

Body_template = skin_dualquat_body(Body_template, transforms2);
for i = 1:21
     Body_template.centers(i,:) = Body_template.spheres{1,i}.center;
end

figure()
hold on;
axis equal
axis off
h = trimesh(Body_template.F, Body_template.V(:, 1), Body_template.V(:, 2), Body_template.V(:, 3), 'EdgeColor', 'none', 'FaceColor', [0.5, 0.5, 0.5], 'FaceAlpha', 1);
lighting gouraud;
view([-90, 0]);
camlight;
view([90, 0]);
camlight;
hold off;

figure()
axis equal
axis off
hold on
scatter3(points.vertices(:,1),points.vertices(:,2),points.vertices(:,3),'.', 'MarkerEdgeColor',[180/255, 180/255, 180/255]);
scatter3(Body_template.V(:,1),Body_template.V(:,2),Body_template.V(:,3),'.', 'MarkerEdgeColor',[190/255, 240/255, 251/255]);
% scatter3(hand_template.centers(:,1),hand_template.centers(:,2),hand_template.centers(:,3),'o','MarkerEdgeColor',[255/255, 0/255, 0/255]);
hold off

%% Things to do

% Apply dual quarternion skinning to root --> pose fitting --> DQS -->
% iteration

%% ICP registration

targetV = points.vertices;
sourceV = Body_template.V;
targetF = points.faces;
sourceF = Body_template.F;
iterations = 30;
flag_prealligndata = 1;
figureOn = 1;
rigidICP = 0;

[sourceV] = ICP_nonrigidICP(targetV, sourceV, targetF, sourceF, iterations, flag_prealligndata, figureOn, rigidICP)

vertices_c = sourceV;
% clear targetV sourceV targetF sourceF 

figure()
axis equal
axis off
hold on
scatter3(points.vertices(:,1),points.vertices(:,2),points.vertices(:,3),'.', 'MarkerEdgeColor',[180/255, 180/255, 180/255]);
scatter3(sourceV(:,1),sourceV(:,2),sourceV(:,3),'.', 'MarkerEdgeColor',[190/255, 240/255, 251/255]);
scatter3(centers_c(:,1),centers_c(:,2),centers_c(:,3),'o','MarkerEdgeColor',[255/255, 0/255, 0/255]);
hold off

%% memo - DQS required variable


transforms_ad{1} = matrix_rotation( ... % D2 MCP -ab/ad
    angle(16), ... % rotation angle: 0 ~ 2, range 3 = 180 deg. 
    matrix_apply(transforms{2}, axes{6}(1 : 3, 3)', 0), ... axis - 3: abduction/adduction
    matrix_apply(transforms{2}, axes{6}(1 : 3, 4)') ... % center
) * transforms{2};



transformed = skin_dualquat(transformed, transforms);

bones.parent
mesh.spheres{i}.center
mesh.vertices = dualquatlbs(mesh.vertices, DQ, mesh.weights);
mesh.normals = dualquatlbs(mesh.normals, DQ_normals, mesh.weights);


for i = 1:18
    mesh.spheres{1,i}.center = centers_c(i,:);
end 



    figure(99)
        h = trisurf(sourceF, sourceV(:, 1), sourceV(:, 2), sourceV(:, 3), 0.3, 'Edgecolor', 'none');
        hold on
        light
        lighting phong;
        set(gca, 'visible', 'off')
        set(gcf, 'Color', [1 1 1])
        view(2)
        set(gca, 'DataAspectRatio', [1 1 1], 'PlotBoxAspectRatio', [1 1 1]);
        tttt = trisurf(targetF, targetV(:, 1), targetV(:, 2), targetV(:, 3), 'Facecolor', 'm', 'Edgecolor', 'none');
        alpha(0.6)

        
%% Unused information

% LMt(1,1:3) = [0, 174, 1708]; %vertex
% LMt(2,1:3) = [-423, 50, 840]; %right hand
% LMt(3,1:3) = [423, 50, 840]; %left hand
% LMt(4,1:3) = [-103, 16, 25]; %right foot
% LMt(5,1:3) = [103, 16, 25]; %left foot
% LMt(6,1:3) = [-2.9,89.7,758.1]; %crotch
% 
% LMs(1,1:3) = [-71, 1683, -33];
% LMs(2,1:3) = [-464, 780 ,15];
% LMs(3,1:3) = [354, 776, 61];
% LMs(4,1:3) = [-235, 11, 82]; 
% LMs(5,1:3) = [77, 0, 105];
% LMs(6,1:3) = [-69.88,714.5,-2.847];