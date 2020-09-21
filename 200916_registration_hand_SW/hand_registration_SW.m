clc
clear all

%% register library - PC
addpath(genpath('../external'));
addpath('D:\GitHub\Hand_deformation_model\functions');
addpath('D:\GitHub\Hand_deformation_model\data_SW');
addpath('D:\GitHub\Hand_deformation_model\data');
addpath('D:\GitHub\Hand_deformation_model\external\registration');

%% register library - Macbook
clc
clear all
addpath(genpath('../external'));
addpath('/Users/hayoungjung/Documents/GitHub/Hand_deformation_model/data');
addpath('/Users/hayoungjung/Documents/GitHub/Hand_deformation_model/data_SW');
addpath('/Users/hayoungjung/Documents/GitHub/Hand_deformation_model/functions');
addpath('/Users/hayoungjung/Documents/GitHub/Hand_deformation_model/external/registration');
format shortG

%% Load data
load('hy_mesh_n4.mat'); %template
LMs = function_get_LM_from_iges('LMs4.igs');
LMs(5,:) = [];
LMt = function_get_LM_from_iges('LMt.igs');
points = {};
% import problem
[points.vertices, points.faces, points.FB, points.H] = function_loading_ply_file('MJ_P04.ply');
points.normals = per_vertex_normals(points.vertices, points.faces);

vertices = mesh.vertices;
faces = mesh.faces;
centers_c = mesh.centers;

%% Rough registration
% size of template vertices = n, % size of scan points = m
n = size(mesh.vertices, 1); 
m = size(points.vertices, 1);
Template_LM = zeros(size(LMs,1),1);

for i=1:size(LMs,1)
delta = mesh.vertices - repmat(LMt(i, :), n, 1);
distances = sum(delta .^ 2, 2);
[~, j] = min(distances);
Template_LM(i,:) = j;
end
% template index based LMt info. update
for i = 1:size(LMs,1)
LMt(i,:) = mesh.vertices(Template_LM(i),:);
end 

LMt = LMt'; LMs = LMs';
clear j delta distances 

% apply transformation matrix
[regParams,Bfit,ErrorStats] = absor(LMt,LMs);
vertices = apply_matrix(regParams.M, vertices, 1);
centers_c = apply_matrix(regParams.M, centers_c, 1);

% generation transformation matrix
transforms = cell(1, 18);
for i = 1 : 18
    transforms{i} = eye(4);
end
axes = compute_bone_axes(mesh.spheres);

normals = per_vertex_normals(vertices, faces);

figure()
trimesh(faces, vertices(:, 1), vertices(:, 2), vertices(:, 3), 'EdgeColor', 'none', 'FaceColor', [0.4, 0.9, 0.4], 'FaceAlpha', 0.5);
hold on;
trimesh(points.faces, points.vertices(:, 1), points.vertices(:, 2), points.vertices(:, 3), 'EdgeColor', 'none', 'FaceColor', [0.8, 0.8, 0.8], 'FaceAlpha', 0.5);
%quiver3(vertices(:, 1), vertices(:, 2), vertices(:, 3), normals(:, 1), normals(:, 2), normals(:, 3), 'Color', [0.4, 0.9, 0.4]);
%quiver3(points.vertices(:, 1), points.vertices(:, 2), points.vertices(:, 3), points.normals(:, 1), points.normals(:, 2), points.normals(:, 3), 'Color', [0.8, 0.8, 0.8]);
hold off;
view([-172,5]);
camlight;
view([90, 0]);
camlight;
view([193, -4]);
axis equal;
grid off;
lighting gouraud;
axis off;
title('initial registration');
% pause;

vertices_c = vertices;
faces_c = faces;


%% palm registration
keep = ismember(mesh.assignment, 1:5);
[vertices, faces] = filter_vertices(vertices, faces, keep);
normals = normals(keep, :);
pairs = compute_correspondences_palm(vertices, normals, points.vertices, points.normals);
figure()
transform = eye(4);
for i = 1 : 10
    delta = compute_transformation(vertices, points.vertices, points.normals, pairs);
    transform = delta * transform;
    vertices = apply_matrix(delta, vertices);
    normals = apply_matrix(delta, normals, 0);
    pairs = compute_correspondences_palm(vertices, normals, points.vertices, points.normals);
    v = get(gca, 'view'); 
    trimesh(faces, vertices(:, 1), vertices(:, 2), vertices(:, 3), 'EdgeColor', 'none', 'FaceColor', [0.4, 0.9, 0.4], 'FaceAlpha', 0.1);
    hold on;
    trimesh(points.faces, points.vertices(:, 1), points.vertices(:, 2), points.vertices(:, 3), 'EdgeColor', 'none', 'FaceColor', [0.8, 0.8, 0.8], 'FaceAlpha', 0.1);
    plot3( ...
        [vertices(pairs(:, 1), 1), points.vertices(pairs(:, 2), 1)]', ...
        [vertices(pairs(:, 1), 2), points.vertices(pairs(:, 2), 2)]', ...
        [vertices(pairs(:, 1), 3), points.vertices(pairs(:, 2), 3)]', ...
    'Color', 'red');
    hold off;
    view([-172,5]);
    camlight;
    view([-172,5]);
    camlight;
    axis equal;
    grid off;
    lighting gouraud;
    axis off;
    title(['After ', num2str(i), ' rigid transformation']);
    set(gca, 'view', v);
    pause(0.01);
end

transforms{2} = transform;

vertices_c = apply_matrix(transform, vertices_c, 1); % update current vertices
centers_c = apply_matrix(transform, centers_c); % update current centers
normals = per_vertex_normals(vertices_c, faces);

vertices_b = vertices_c; % vertices_b = vertices after palm registration
centers_b = centers_c; % centers_b = centers after palm registration

hand_template = mesh;
for i = 1:18
    hand_template.spheres{1,i}.center = centers_c(i,:);
end
hand_template.vertices = vertices_c;
hand_template.normals = per_vertex_normals(hand_template.vertices, hand_template.faces);

clear Bfit ErrorStats LMs LMt regParams Template_LM

figure()
hold on;
axis equal
axis off
h = trimesh(hand_template.faces, hand_template.vertices(:, 1), hand_template.vertices(:, 2), hand_template.vertices(:, 3), 'EdgeColor', 'none', 'FaceColor', [0.5, 0.5, 0.5], 'FaceAlpha', 1);
lighting gouraud;
view([193, -4]);
camlight;
view([185, 8]);
camlight;
hold off;

%% Posture Calculation

LMs = function_get_LM_from_iges('LMs4_n.igs');
LMt = function_get_LM_from_iges('LMt_new.igs');
Template_LM = zeros(24,1);
m = size(hand_template.vertices, 1);

for i=1:24
delta = hand_template.vertices - repmat(LMt(i, :), m, 1);
distances = sum(delta .^ 2, 2);
[~, j] = min(distances);
Template_LM(i,:) = j;
end
for i = 1:24
LMt(i,:) = hand_template.vertices(Template_LM(i),:);
end 

idx_LMt_24 = Template_LM;

Tcor = zeros(12,3);
Scor = zeros(13,3);
IDX = [1:3, 7:9, 13:15, 19:21];

for xx = 1:12
    Tcor(xx,:) = (LMt(IDX(xx),:) + LMt(IDX(xx)+3,:))/2;
    Scor(xx,:) = (LMs(IDX(xx),:) + LMs(IDX(xx)+3,:))/2;
end

% Input wrist center CoR
Scor(13,:) = hand_template.spheres{1,22}.center;
Tcor(13,:) = hand_template.spheres{1,22}.center;

% angle calculation for Scor, Tcor = Things to do - develop as function
o = [0 0 0];
x = [1 0 0];
y = [0 1 0];
z = [0 0 1];

% generate vectors for angle calculation
% D3 MCP case
v_T_D3_MCB = (hand_template.spheres{1,12}.center - hand_template.spheres{1,22}.center)/norm((hand_template.spheres{1,12}.center - hand_template.spheres{1,22}.center));
v_T_D3_PPLX = (hand_template.spheres{1,11}.center - hand_template.spheres{1,12}.center)/norm((hand_template.spheres{1,11}.center - hand_template.spheres{1,12}.center));
v_S_D3_MCB = (Scor(4,:) - Scor(13,:))/norm(Scor(4,:) - Scor(13,:));
v_S_D3_PPLX = (Scor(5,:) - Scor(4,:))/norm(Scor(5,:) - Scor(4,:));
D3_MCB_axis = axes{9}(1 : 3, 3)';

AXIS_D3 = [o; v_S_D3_MCB; v_S_D3_PPLX; D3_MCB_axis; x; y; z];

% Z-axis rotation
vectorK = D3_MCB_axis(1,1:2);
vectorAxis = [0, 1]
rotationAngle_Z = acosd(dot(vectorK, vectorAxis) / (norm(vectorK) * norm(vectorAxis)));
Rz = function_rotationmat3D((-rotationAngle_Z)/180*pi, [0, 0, 1]);
D3_MCB_axis = function_rotation_matrix(D3_MCB_axis, Rz);
v_S_D3_MCB = function_rotation_matrix(v_S_D3_MCB, Rz);
v_S_D3_PPLX = function_rotation_matrix(v_S_D3_PPLX, Rz);
%update AXIS
AXIS_D3 = [o; v_S_D3_MCB; v_S_D3_PPLX; D3_MCB_axis; x; y; z];
                                    
% X-axis rotation
vectorK = D3_MCB_axis(1,2:3);
vectorAxis = [0, 1]
rotationAngle_X = acosd(dot(vectorK, vectorAxis) / (norm(vectorK) * norm(vectorAxis)));
Rx = function_rotationmat3D((rotationAngle_X)/180*pi, [1, 0, 0]);
D3_MCB_axis = function_rotation_matrix(D3_MCB_axis, Rx);
v_S_D3_MCB = function_rotation_matrix(v_S_D3_MCB, Rx);
v_S_D3_PPLX = function_rotation_matrix(v_S_D3_PPLX, Rx);
%update AXIS
AXIS_D3 = [o; v_S_D3_MCB; v_S_D3_PPLX; D3_MCB_axis; x; y; z];

%MCB axis = blue, MCB bone = blue, PPLX bone = Red, 
% figure()
% axis equal
% hold on
% scatter3(AXIS_D4([1 5:7],1),AXIS_D4([1 5:7],2),AXIS_D4([1 5:7],3),'*','MarkerEdgeColor',[0/255, 0/255, 0/255]);
% scatter3(AXIS_D4([2 4],1),AXIS_D4([2 4],2),AXIS_D4([2 4],3),'*','MarkerEdgeColor',[0/255, 0/255, 255/255]);
% scatter3(AXIS_D4(3,1),AXIS_D4(3,2),AXIS_D4(3,3),'*', 'MarkerEdgeColor',[255/255, 0/255, 0/255]);
% plot3(AXIS_D4(1:2,1),AXIS_D4(1:2,2),AXIS_D4(1:2,3),'-g');
% plot3(AXIS_D4([1 3],1),AXIS_D4([1 3],2),AXIS_D4([1 3],3),'-r');
% plot3(AXIS_D4([1 4],1),AXIS_D4([1 4],2),AXIS_D4([1 4],3),'-b');
% plot3(AXIS_D4([1 5],1),AXIS_D4([1 5],2),AXIS_D4([1 5],3),'-k');
% plot3(AXIS_D4([1 6],1),AXIS_D4([1 6],2),AXIS_D4([1 6],3),'-k');
% plot3(AXIS_D4([1 7],1),AXIS_D4([1 7],2),AXIS_D4([1 7],3),'-k');
% hold off

vectorK = AXIS_D3(2,1:2)
vectorJ = AXIS_D3(3,1:2)
s_angle_bt_MCB_PPLX = acosd(dot(vectorK, vectorJ) / (norm(vectorK) * norm(vectorJ)));


% angle calculation for Tcor
% Z-axis rotation
v_T_D3_MCB = function_rotation_matrix(v_T_D3_MCB, Rz);
v_T_D3_PPLX = function_rotation_matrix(v_T_D3_PPLX, Rz);
                                    
% X-axis rotation
v_T_D3_MCB = function_rotation_matrix(v_T_D3_MCB, Rx);
v_T_D3_PPLX = function_rotation_matrix(v_T_D3_PPLX, Rx);
% update AXIS value
AXIS_D3 = [o; v_T_D3_MCB; v_T_D3_PPLX; D3_MCB_axis; x; y; z]

vectorK = AXIS_D3(2,1:2)
vectorJ = AXIS_D3(3,1:2)
t_angle_bt_MCB_PPLX = acosd(dot(vectorK, vectorJ) / (norm(vectorK) * norm(vectorJ)));

AXIS1 = [o; v_S_D3_MCB; v_S_D3_PPLX; D3_MCB_axis; x; y; z];
AXIS2 = [o; v_T_D3_MCB; v_T_D3_PPLX; D3_MCB_axis; x; y; z];

% figure()
% axis equal
% hold on
% scatter3(AXIS1([1 5:7],1),AXIS1([1 5:7],2),AXIS1([1 5:7],3),'*','MarkerEdgeColor',[0/255, 0/255, 0/255]);
% scatter3(AXIS1([2 4],1),AXIS1([2 4],2),AXIS1([2 4],3),'*','MarkerEdgeColor',[255/255, 0/255, 0/255]);
% scatter3(AXIS2([2 4],1),AXIS2([2 4],2),AXIS2([2 4],3),'*','MarkerEdgeColor',[0/255, 0/255, 255/255]);
% scatter3(AXIS1(3,1),AXIS1(3,2),AXIS1(3,3),'*', 'MarkerEdgeColor',[255/255, 0/255, 0/255]);
% scatter3(AXIS2(3,1),AXIS2(3,2),AXIS2(3,3),'*', 'MarkerEdgeColor',[0/255, 0/255, 255/255]);
% plot3(AXIS1(1:2,1),AXIS1(1:2,2),AXIS1(1:2,3),'-r');
% plot3(AXIS2(1:2,1),AXIS2(1:2,2),AXIS2(1:2,3),'-b');
% plot3(AXIS1([1 3],1),AXIS1([1 3],2),AXIS1([1 3],3),'-r');
% plot3(AXIS2([1 3],1),AXIS2([1 3],2),AXIS2([1 3],3),'-b');
% plot3(AXIS1([1 4],1),AXIS1([1 4],2),AXIS1([1 4],3),'-g');
% plot3(AXIS1([1 5],1),AXIS1([1 5],2),AXIS1([1 5],3),'-k');
% plot3(AXIS1([1 6],1),AXIS1([1 6],2),AXIS1([1 6],3),'-k');
% plot3(AXIS1([1 7],1),AXIS1([1 7],2),AXIS1([1 7],3),'-k');
% hold off

angle_diff_D3 = abs(t_angle_bt_MCB_PPLX - s_angle_bt_MCB_PPLX)

% D4 MCP case
v_T_D4_MCB = (hand_template.spheres{1,8}.center - hand_template.spheres{1,22}.center)/norm((hand_template.spheres{1,8}.center - hand_template.spheres{1,22}.center));
v_T_D4_PPLX = (hand_template.spheres{1,7}.center - hand_template.spheres{1,8}.center)/norm((hand_template.spheres{1,7}.center - hand_template.spheres{1,8}.center));
v_S_D4_MCB = (Scor(7,:) - Scor(13,:))/norm(Scor(7,:) - Scor(13,:));
v_S_D4_PPLX = (Scor(8,:) - Scor(7,:))/norm(Scor(8,:) - Scor(7,:));
D4_MCB_axis = axes{12}(1 : 3, 3)';
AXIS_D4 = [o; v_S_D4_MCB; v_S_D4_PPLX; D4_MCB_axis; x; y; z];

% Z-axis rotation
vectorK = D4_MCB_axis(1,1:2);
vectorAxis = [0, 1]
rotationAngle_Z = acosd(dot(vectorK, vectorAxis) / (norm(vectorK) * norm(vectorAxis)));
Rz = function_rotationmat3D((-rotationAngle_Z)/180*pi, [0, 0, 1]);
D4_MCB_axis = function_rotation_matrix(D4_MCB_axis, Rz);
v_S_D4_MCB = function_rotation_matrix(v_S_D4_MCB, Rz);
v_S_D4_PPLX = function_rotation_matrix(v_S_D4_PPLX, Rz);
%update AXIS
AXIS_D4 = [o; v_S_D4_MCB; v_S_D4_PPLX; D4_MCB_axis; x; y; z];
                                    
% X-axis rotation
vectorK = D4_MCB_axis(1,2:3);
vectorAxis = [0, 1]
rotationAngle_X = acosd(dot(vectorK, vectorAxis) / (norm(vectorK) * norm(vectorAxis)));
Rx = function_rotationmat3D((rotationAngle_X)/180*pi, [1, 0, 0]);
D4_MCB_axis = function_rotation_matrix(D4_MCB_axis, Rx);
v_S_D4_MCB = function_rotation_matrix(v_S_D4_MCB, Rx);
v_S_D4_PPLX = function_rotation_matrix(v_S_D4_PPLX, Rx);
%update AXIS
AXIS_D4 = [o; v_S_D4_MCB; v_S_D4_PPLX; D4_MCB_axis; x; y; z];

vectorK = AXIS_D4(2,1:2)
vectorJ = AXIS_D4(3,1:2)
s_angle_bt_MCB_PPLX = acosd(dot(vectorK, vectorJ) / (norm(vectorK) * norm(vectorJ)));

% angle calculation for Tcor
% Z-axis rotation
v_T_D4_MCB = function_rotation_matrix(v_T_D4_MCB, Rz);
v_T_D4_PPLX = function_rotation_matrix(v_T_D4_PPLX, Rz);
                                    
% X-axis rotation
v_T_D4_MCB = function_rotation_matrix(v_T_D4_MCB, Rx);
v_T_D4_PPLX = function_rotation_matrix(v_T_D4_PPLX, Rx);
% update AXIS value
AXIS_D4 = [o; v_T_D4_MCB; v_T_D4_PPLX; D4_MCB_axis; x; y; z];

vectorK = AXIS_D4(2,1:2)
vectorJ = AXIS_D4(3,1:2)
t_angle_bt_MCB_PPLX = acosd(dot(vectorK, vectorJ) / (norm(vectorK) * norm(vectorJ)));

angle_diff_D4 = abs(t_angle_bt_MCB_PPLX - s_angle_bt_MCB_PPLX)

% D5 MCP case
v_T_D5_MCB = (hand_template.spheres{1,4}.center - hand_template.spheres{1,22}.center)/norm((hand_template.spheres{1,4}.center - hand_template.spheres{1,22}.center));
v_T_D5_PPLX = (hand_template.spheres{1,3}.center - hand_template.spheres{1,4}.center)/norm((hand_template.spheres{1,3}.center - hand_template.spheres{1,4}.center));
v_S_D5_MCB = (Scor(10,:) - Scor(13,:))/norm(Scor(10,:) - Scor(13,:));
v_S_D5_PPLX = (Scor(11,:) - Scor(10,:))/norm(Scor(11,:) - Scor(10,:));
D5_MCB_axis = axes{15}(1 : 3, 3)';
AXIS_D5 = [o; v_S_D5_MCB; v_S_D5_PPLX; D5_MCB_axis; x; y; z];

% Z-axis rotation
vectorK = D5_MCB_axis(1,1:2);
vectorAxis = [0, 1]
rotationAngle_Z = acosd(dot(vectorK, vectorAxis) / (norm(vectorK) * norm(vectorAxis)));
Rz = function_rotationmat3D((-rotationAngle_Z)/180*pi, [0, 0, 1]);
D5_MCB_axis = function_rotation_matrix(D5_MCB_axis, Rz);
v_S_D5_MCB = function_rotation_matrix(v_S_D5_MCB, Rz);
v_S_D5_PPLX = function_rotation_matrix(v_S_D5_PPLX, Rz);
%update AXIS
AXIS_D5 = [o; v_S_D5_MCB; v_S_D5_PPLX; D5_MCB_axis; x; y; z];
                                    
% X-axis rotation
vectorK = D5_MCB_axis(1,2:3);
vectorAxis = [0, 1]
rotationAngle_X = acosd(dot(vectorK, vectorAxis) / (norm(vectorK) * norm(vectorAxis)));
Rx = function_rotationmat3D((rotationAngle_X)/180*pi, [1, 0, 0]);
D5_MCB_axis = function_rotation_matrix(D5_MCB_axis, Rx);
v_S_D5_MCB = function_rotation_matrix(v_S_D5_MCB, Rx);
v_S_D5_PPLX = function_rotation_matrix(v_S_D5_PPLX, Rx);
%update AXIS
AXIS_D5 = [o; v_S_D5_MCB; v_S_D5_PPLX; D5_MCB_axis; x; y; z];

vectorK = AXIS_D5(2,1:2)
vectorJ = AXIS_D5(3,1:2)
s_angle_bt_MCB_PPLX = acosd(dot(vectorK, vectorJ) / (norm(vectorK) * norm(vectorJ)));

% angle calculation for Tcor
% Z-axis rotation
v_T_D5_MCB = function_rotation_matrix(v_T_D5_MCB, Rz);
v_T_D5_PPLX = function_rotation_matrix(v_T_D5_PPLX, Rz);
                                    
% X-axis rotation
v_T_D5_MCB = function_rotation_matrix(v_T_D5_MCB, Rx);
v_T_D5_PPLX = function_rotation_matrix(v_T_D5_PPLX, Rx);
% update AXIS value
AXIS_D5 = [o; v_T_D5_MCB; v_T_D5_PPLX; D5_MCB_axis; x; y; z];

vectorK = AXIS_D5(2,1:2)
vectorJ = AXIS_D5(3,1:2)
t_angle_bt_MCB_PPLX = acosd(dot(vectorK, vectorJ) / (norm(vectorK) * norm(vectorJ)));

angle_diff_D5 = abs(t_angle_bt_MCB_PPLX - s_angle_bt_MCB_PPLX)

%% Posture change of MCP bone

transforms = cell(1, 18);
for i = 1 : 18
    transforms{i} = eye(4);
end
axes = bone_axes(mesh.spheres);

transforms_ad = cell(1,4);
for i = 1 : 4
    transforms_ad{i} = eye(4);
end 

 angle = zeros(19,1);
 angle(16) = 0/60; %D2
 angle(17) = angle_diff_D3/60; %D3
 angle(18) = angle_diff_D4/60; %D4
 angle(19) = angle_diff_D5/60; %D5

 % MCP abduction/adduction motion
transforms_ad{1} = matrix_rotation( ... % D2 MCP -ab/ad
    angle(16), ... % rotation angle: 0 ~ 2, range 3 = 180 deg. 
    matrix_apply(transforms{2}, axes{6}(1 : 3, 3)', 0), ... axis - 3: abduction/adduction
    matrix_apply(transforms{2}, axes{6}(1 : 3, 4)') ... % center
) * transforms{2};
transforms_ad{2} = matrix_rotation( ...  % D3 MCP - ab/ad
    angle(17), ... % rotation angle: 0 ~ 2, range 3 = 180 deg. 
    matrix_apply(transforms{2}, axes{9}(1 : 3, 3)', 0), ... axis - 3: abduction/adduction
    matrix_apply(transforms{2}, axes{9}(1 : 3, 4)') ... % center
) * transforms{2};
transforms_ad{3} = matrix_rotation( ...  % D4 MCP -ab/ad
    angle(18), ... % rotation angle: 0 ~ 2, range 3 = 180 deg. 
    matrix_apply(transforms{2}, axes{12}(1 : 3, 3)', 0), ... axis - 3: abduction/adduction
    matrix_apply(transforms{2}, axes{12}(1 : 3, 4)') ... % center
) * transforms{2};
transforms_ad{4} = matrix_rotation( ... % D5 MCP - ab/ad
    angle(19), ... % rotation angle: 0 ~ 2, range 3 = 180 deg. 
    matrix_apply(transforms{2}, axes{16}(1 : 3, 3)', 0), ...  axis - 3: abduction/adduction
    matrix_apply(transforms{2}, axes{16}(1 : 3, 4)') ... % center
) * transforms{2};

% Flexion/extension of fingers 
transforms{3} = matrix_rotation( ... % D1 CMC
    angle(1), ... % rotation angle: 0 ~ 2, range 3 = 180 deg.
    matrix_apply(transforms{2}, axes{3}(1 : 3, 1)', 0), ... % axis1: red; axis2: green; axis3: blue
    matrix_apply(transforms{2}, axes{3}(1 : 3, 4)') ... % center
) * transforms{2};
transforms{4} = matrix_rotation( ... % D1 MCP
    angle(2), ...
    matrix_apply(transforms{3}, axes{4}(1 : 3, 2)', 0), ... % axis1: red; axis2: green; axis3: blue
    matrix_apply(transforms{3}, axes{4}(1 : 3, 4)') ...
) * transforms{3};
transforms{5} = matrix_rotation( ... % D1 IP
    angle(3), ...
    matrix_apply(transforms{4}, axes{5}(1 : 3, 2)', 0), ... % axis1: red; axis2: green; axis3: blue
    matrix_apply(transforms{4}, axes{5}(1 : 3, 4)') ...
) * transforms{4};

transforms{6} = matrix_rotation( ... % D2 MCP
    angle(4), ... % rotation angle: 0 ~ 2, range 3 = 180 deg. 
    matrix_apply(transforms{2}, axes{6}(1 : 3, 2)', 0), ... % axis - 1: supination, pronation; 2: flexion/extension; 3: abduction/adduction
    matrix_apply(transforms{2}, axes{6}(1 : 3, 4)') ... % center
) * transforms_ad{1} * transforms{2};
transforms{7} = matrix_rotation( ... % D2 PIP
    angle(5), ...
    matrix_apply(transforms{6}, axes{7}(1 : 3, 2)', 0), ...
    matrix_apply(transforms{6}, axes{7}(1 : 3, 4)') ...
) * transforms{6};
transforms{8} = matrix_rotation( ... % D2 DIP
    angle(6), ...
    matrix_apply(transforms{7}, axes{8}(1 : 3, 2)', 0), ...
    matrix_apply(transforms{7}, axes{8}(1 : 3, 4)') ...
) * transforms{7};

transforms{9} = matrix_rotation( ...  % D3 MCP
    angle(7), ... % rotation angle: 0 ~ 2, range 3 = 180 deg. 
    matrix_apply(transforms{2}, axes{9}(1 : 3, 2)', 0), ... % axis - 1: supination, pronation; 2: flexion/extension; 3: abduction/adduction
    matrix_apply(transforms{2}, axes{9}(1 : 3, 4)') ... % center
) * transforms_ad{2} * transforms{2};
transforms{10} = matrix_rotation( ... % D3 PIP
    angle(8), ...
    matrix_apply(transforms{9}, axes{10}(1 : 3, 2)', 0), ...
    matrix_apply(transforms{9}, axes{10}(1 : 3, 4)') ...
) * transforms{9};
transforms{11} = matrix_rotation( ... % D3 DIP
    angle(9), ...
    matrix_apply(transforms{10}, axes{11}(1 : 3, 2)', 0), ...
    matrix_apply(transforms{10}, axes{11}(1 : 3, 4)') ...
) * transforms{10};

transforms{12} = matrix_rotation( ...  % D4 MCP
    angle(10), ... % rotation angle: 0 ~ 2, range 3 = 180 deg. 
    matrix_apply(transforms{2}, axes{12}(1 : 3, 2)', 0), ... % axis - 1: supination, pronation; 2: flexion/extension; 3: abduction/adduction
    matrix_apply(transforms{2}, axes{12}(1 : 3, 4)') ... % center
) * transforms_ad{3}* transforms{2};
transforms{13} = matrix_rotation( ... % D4 PIP
    angle(11), ...
    matrix_apply(transforms{12}, axes{13}(1 : 3, 2)', 0), ...
    matrix_apply(transforms{12}, axes{13}(1 : 3, 4)') ...
) * transforms{12};
transforms{14} = matrix_rotation( ... % D4 DIP
    angle(12), ...
    matrix_apply(transforms{13}, axes{14}(1 : 3, 2)', 0), ...
    matrix_apply(transforms{13}, axes{14}(1 : 3, 4)') ...
) * transforms{13};

transforms{16} = matrix_rotation( ... % D5 MCP
    angle(13), ... % rotation angle: 0 ~ 2, range 3 = 180 deg. 
    matrix_apply(transforms{2}, axes{16}(1 : 3, 2)', 0), ... % axis - 1: supination, pronation; 2: flexion/extension; 3: abduction/adduction
    matrix_apply(transforms{2}, axes{16}(1 : 3, 4)') ... % center
) * transforms_ad{4} * transforms{2};
transforms{17} = matrix_rotation( ...% D5 PIP
    angle(14), ...
    matrix_apply(transforms{16}, axes{17}(1 : 3, 2)', 0), ...
    matrix_apply(transforms{16}, axes{17}(1 : 3, 4)') ...
) * transforms{16};
transforms{18} = matrix_rotation( ... % D5 DIP
    angle(15), ...
    matrix_apply(transforms{17}, axes{18}(1 : 3, 2)', 0), ...
    matrix_apply(transforms{17}, axes{18}(1 : 3, 4)') ...
) * transforms{17};

%Apply new hand posture by angle & render 3D model   

hand_template = skin_dualquat(hand_template, transforms);
for i = 1:18
     hand_template.centers(i,:) = hand_template.spheres{1,i}.center;
end

figure()
hold on;
axis equal
axis off
h = trimesh(hand_template.faces, hand_template.vertices(:, 1), hand_template.vertices(:, 2), hand_template.vertices(:, 3), 'EdgeColor', 'none', 'FaceColor', [0.5, 0.5, 0.5], 'FaceAlpha', 1);
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
scatter3(hand_template.vertices(:,1),hand_template.vertices(:,2),hand_template.vertices(:,3),'.', 'MarkerEdgeColor',[190/255, 240/255, 251/255]);
% scatter3(hand_template.centers(:,1),hand_template.centers(:,2),hand_template.centers(:,3),'o','MarkerEdgeColor',[255/255, 0/255, 0/255]);
hold off

vertices_c = hand_template.vertices;
normals = hand_template.normals;

%% parameter for finger root (MCP) registration 

FRP_segment = [6 9 12 15 18];
FRP_cor = [20 16 12 8 4];
FRP_digits{1} = [6:8];
FRP_digits{2} = [9:11];
FRP_digits{3} = [12:14];
FRP_digits{4} = [15:17];
FRP_digits{5} = [18:20];

FRP_cor_tr{1} = [17:19];
FRP_cor_tr{2} = [13:15];
FRP_cor_tr{3} = [9:11];
FRP_cor_tr{4} = [5:7];
FRP_cor_tr{5} = [1:3];

transform_order= [3 6 9 12 16];

%% D1-D5 finger root (MCP) registration
h3 = [];
h4 = [];

for j = 1:5

vertices = vertices_c;
faces = faces_c;
normals = per_vertex_normals(vertices, faces);

keep = ismember(mesh.assignment, FRP_segment(j));
[vertices, faces] = filter_vertices(vertices, faces, keep);
normals = normals(keep, :);
pairs = compute_correspondences_MCP(vertices, normals, points.vertices, points.normals);

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
        pairs = compute_correspondences_MCP(vertices, normals, points.vertices, points.normals);
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
   
keep = ismember(mesh.assignment, FRP_digits{j});
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
% D1 CMC
transforms{3} = transforms{3};
% D1 MCP
transforms{4} = transforms{4} * transforms{3};
% D1 IP
transforms{5} = transforms{5} * transforms{4};
% D2 MCP
transforms{6} = transforms{6};
% D2 PIP
transforms{7} = transforms{7} * transforms{6};
% D2 DIP
transforms{8} = transforms{8} * transforms{7};
% D3 MCP
transforms{9} = transforms{9};
% D3 PIP
transforms{10} = transforms{10} * transforms{9};
% D3 DIP
transforms{11} = transforms{11} * transforms{10};
% D4 MCP
transforms{12} = transforms{12};
% D4 PIP
transforms{13} = transforms{13} * transforms{12};
% D4 DIP
transforms{14} = transforms{14}  * transforms{13};
% D5 MCP
transforms{16} = transforms{16};
% D5 PIP
transforms{17} = transforms{17} * transforms{16};
% D5 DIP
transforms{18} = transforms{18} * transforms{17};

%% DQS application & Result display

hand_template = skin_dualquat(hand_template, transforms);
for i = 1:18
     hand_template.centers(i,:) = hand_template.spheres{1,i}.center;
end

figure()
hold on;
axis equal
axis off
h = trimesh(hand_template.faces, hand_template.vertices(:, 1), hand_template.vertices(:, 2), hand_template.vertices(:, 3), 'EdgeColor', 'none', 'FaceColor', [0.5, 0.5, 0.5], 'FaceAlpha', 1);
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
scatter3(hand_template.vertices(:,1),hand_template.vertices(:,2),hand_template.vertices(:,3),'.', 'MarkerEdgeColor',[190/255, 240/255, 251/255]);
% scatter3(hand_template.centers(:,1),hand_template.centers(:,2),hand_template.centers(:,3),'o','MarkerEdgeColor',[255/255, 0/255, 0/255]);
hold off

vertices_c = hand_template.vertices;
normals = hand_template.normals;

%% parameter for fingers registration order

P_segment = [7 8 10 11 13 14 16 17 19 20];
P_cor = [19 18 15 14 11 10 7 6 3 2];

P_digits{1} = [7:8];
P_digits{2} = 8;
P_digits{3} = [10:11];
P_digits{4} = 11;
P_digits{5} = [13:14];
P_digits{6} = 14;
P_digits{7} = [16:17];
P_digits{8} = 17;
P_digits{9} = [19:20];
P_digits{10} = 20;

P_cor_tr{1} = [17:18];
P_cor_tr{2} = 17;
P_cor_tr{3} = [13:14];
P_cor_tr{4} = 13;
P_cor_tr{5} = [9:10];
P_cor_tr{6} = 9;
P_cor_tr{7} = [5:6];
P_cor_tr{8} = 5;
P_cor_tr{9} = [1:2];
P_cor_tr{10} = 1;

transform_order= [4 5 7 8 10 11 13 14 17 18];

%% D1-D5 MCP,PIP,DID registration
h3 = [];
h4 = [];

transforms2 = cell(1, 18);
for i = 1 : 18
    transforms2{i} = eye(4);
end

for j = 1:10

vertices = vertices_c;
faces = faces_c;
normals = per_vertex_normals(vertices, faces);

keep = ismember(mesh.assignment, P_segment(j));
[vertices, faces] = filter_vertices(vertices, faces, keep);
normals = normals(keep, :);
pairs = compute_correspondences(vertices, normals, points.vertices, points.normals);

transform = eye(4);

figure(2)
    for i = 1 : 13
        delete(h3);
        delete(h4);
        delta = compute_transformation(vertices, points.vertices, points.normals, pairs);
        delta = constraint_transformation(delta, centers_c(P_cor(j),:));
        transform = delta * transform;
        vertices = apply_matrix(delta, vertices);
        normals = apply_matrix(delta, normals, 0);
        pairs = compute_correspondences(vertices, normals, points.vertices, points.normals);
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
   
keep = ismember(mesh.assignment, P_digits{j});
vi_Dx = vertices_c(keep,:);
vi_Dx = apply_matrix(transform, vi_Dx, 1);

vertices_c(keep,:) = vi_Dx;
centers_c(P_cor_tr{j},:) = apply_matrix(transform, centers_c(P_cor_tr{j},:), 1);
normals = per_vertex_normals(vertices_c, faces);

end

%% 
% D1 MCP
transforms2{4} = transforms2{4} * transforms2{3};
% D1 IP
transforms2{5} = transforms2{5} * transforms2{4};
% D2 PIP

transforms2{7} = transforms2{7} * transforms2{6};
% D2 DIP
transforms2{8} = transforms2{8} * transforms2{7};
% D3 PIP
transforms2{10} = transforms2{10} * transforms2{9};
% D3 DIP
transforms2{11} = transforms2{11} * transforms2{10};
% D4 PIP
transforms2{13} = transforms2{13} * transforms2{12};
% D4 DIP
transforms2{14} = transforms2{14}  * transforms2{13};
% D5 PIP
transforms2{17} = transforms2{17} * transforms2{16};
% D5 DIP
transforms2{18} = transforms2{18} * transforms2{17};


%% DQS application & Result display

hand_template = skin_dualquat(hand_template, transforms2);
for i = 1:18
     hand_template.centers(i,:) = hand_template.spheres{1,i}.center;
end

figure()
hold on;
axis equal
axis off
h = trimesh(hand_template.faces, hand_template.vertices(:, 1), hand_template.vertices(:, 2), hand_template.vertices(:, 3), 'EdgeColor', 'none', 'FaceColor', [0.5, 0.5, 0.5], 'FaceAlpha', 1);
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
scatter3(hand_template.vertices(:,1),hand_template.vertices(:,2),hand_template.vertices(:,3),'.', 'MarkerEdgeColor',[190/255, 240/255, 251/255]);
% scatter3(hand_template.centers(:,1),hand_template.centers(:,2),hand_template.centers(:,3),'o','MarkerEdgeColor',[255/255, 0/255, 0/255]);
hold off

%% Things to do

% Apply dual quarternion skinning to root --> pose fitting --> DQS -->
% iteration

%% ICP registration

targetV = points.vertices;
sourceV = hand_template.vertices;
targetF = points.faces;
sourceF = hand_template.faces;
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

%% save mat file
% V_P6 = sourceV;
% save P6_vertices.mat V_P6

% save function 
% Header »ý¼º
% % load('THD.mat'); 
% THD = cell(10,1);
% THD{1} = ['ply', newline()];
% THD{2} = ['format ascii 1.0', newline()];
% THD{3} = ['comment Exported by RapidForm', newline()];
% THD{4} = ['element vertex 6984', newline()];
% THD{5} = ['property float x', newline()];
% THD{6} = ['property float y', newline()];
% THD{7} = ['property float z', newline()];
% THD{8} = ['element face 13964', newline()];
% THD{9} = ['property list uchar int vertex_index', newline()];
% THD{10} = ['end_header', newline()];
% 
% TF = faces;
% 
% TF2 = zeros(13964,3);
% TF2(:,2:4) = TF;
% TF2(:,1) = 3;
% % 
% function_saving_ply_file(sourceV, TF2, points.H, 'MJ_P03_alinged.ply')










