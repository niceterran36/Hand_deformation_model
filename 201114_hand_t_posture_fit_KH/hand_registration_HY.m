clc
clear all

%% register library - PC Home
addpath(genpath('../external'));
addpath('D:\GitHub\Hand_deformation_model\functions');
addpath('Data');
addpath('D:\GitHub\Hand_deformation_model\201026_hand_t_update');
addpath('D:\GitHub\Hand_deformation_model\external\registration');

%% register library - PC lab
addpath(genpath('../external'));
addpath('F:\[GitHub]\Hand_deformation_model\functions');
addpath('F:\[GitHub]\Hand_deformation_model\data_SW');
addpath('Data');
addpath('F:\[GitHub]\Hand_deformation_model\201026_hand_t_update');
addpath('F:\[GitHub]\Hand_deformation_model\data');
addpath('F:\[GitHub]\Hand_deformation_model\external\registration');
% 
% %% register library - Macbook
% clc
% clear all
% addpath(genpath('../external'));
% addpath('/Users/hayoungjung/Documents/GitHub/Hand_deformation_model/data');
% addpath('/Users/hayoungjung/Documents/GitHub/Hand_deformation_model/data_SW');
% addpath('Data');
% addpath('/Users/hayoungjung/Documents/GitHub/Hand_deformation_model/functions');
% addpath('/Users/hayoungjung/Documents/GitHub/Hand_deformation_model/external/registration');
% format shortG

%% Load data
global assignment_new
load('hy_mesh_n5.mat'); %template
load('assignment_new.mat');
[points.vertices, points.faces, points.FB, points.H] = function_loading_ply_file('HY_pos2.ply'); % target scan
points.normals = per_vertex_normals(points.vertices, points.faces);

%% search template LM index
LMt = function_get_LM_from_iges('template_LMs.igs');
LMs = function_get_LM_from_iges('HY_pos2.igs');
LMt_Idx = zeros(size(LMt,1),1);
m = size(mesh.vertices, 1);
for i = 1:size(LMt,1)
delta = mesh.vertices - repmat(LMt(i, :), m, 1);
distances = sum(delta .^ 2, 2);
[~, j] = min(distances);
LMt_Idx(i,:) = j;
end
for i = 1:size(LMt,1)
LMt(i,:) = mesh.vertices(LMt_Idx(i),:);
end
clear m delta distances i j 

%% palm fitting

% palm scale 
% Landmarks for palm alignment & hand scale
LMs_PLM = function_get_LM_from_iges('HY_pos2_PLM.igs');
LMt_PLM = function_get_LM_from_iges('LMt_PLM.igs');
LMs_PLM = LMs_PLM'; LMt_PLM = LMt_PLM';
[regParams,~,~] = absor(LMt_PLM,LMs_PLM);

% update vertices & CoR
mesh.vertices = apply_matrix(regParams.M, mesh.vertices, 1);
for i = 1:22
    mesh.spheres{1,i}.center = apply_matrix(regParams.M, mesh.spheres{1,i}.center, 1);
    mesh.centers(i,:) = mesh.spheres{1,i}.center;
end

% palm registration
keep = ismember(mesh.assignment, 1:5);
[vertices, faces] = filter_vertices(mesh.vertices, mesh.faces, keep);
normals = per_vertex_normals(mesh.vertices, mesh.faces);
normals = normals(keep, :);
pairs = compute_correspondences_palm(vertices, normals, points.vertices, points.normals);
view_angle = [207,10];
figure()
view(view_angle);
transform = eye(4);
for i = 1 : 5
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
    view([-90,0]);
    camlight;
    view([90,0]);
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

% update vertices & CoR
mesh.vertices = apply_matrix(transform, mesh.vertices, 1); % update current vertices
mesh.normals = per_vertex_normals(mesh.vertices, faces);
for i = 1:22
    mesh.spheres{1,i}.center = apply_matrix(transform, mesh.spheres{1,i}.center,1);
    mesh.centers(i,:) = mesh.spheres{1,i}.center;
end

clear Bfit ErrorStats regParams delta faces h i keep LMs_PLM LMt_PLM pairs transform transforms

figure()
hold on;
axis equal
axis off
h = trimesh(mesh.faces, mesh.vertices(:, 1), mesh.vertices(:, 2), mesh.vertices(:, 3), 'EdgeColor', 'none', 'FaceColor', [0.5, 0.5, 0.5], 'FaceAlpha', 1);
lighting gouraud;
view([193, -4]);
camlight;
view([185, 8]);
camlight;
hold off;

%% segment scale factor


%% segment scale
[transformed] = segment_scale_fingers_new(mesh, LMt, LMs);
mesh = transformed;
clear transformed

mesh_BackUp = mesh;
%mesh = mesh_BackUp;

%% calculate optimal rotation angle for MCP & deformation
% FRP_dorsal_segment = [6 109 112 115 118];

% function X = (segment, mesh.v, points.v) ==> Y = (angle X)
% function X2 = (angle1, angle2) ==> Y = mean distance
% function find_opt_rotation

% D2 MCP
FRP_dorsal_segment = 109;
record = [];
tic
for ag1 = -0.25:0.05:0.25
    for ag2 = 0:0.05:1
        angle = zeros(19,1);
        angle(4) = ag2;
        angle(16) = ag1;
        tr_mesh = transform_angle(mesh, angle);
        mean_distance = mean_dist_tester(tr_mesh, points, FRP_dorsal_segment, assignment_new);
        record = [record; ag1 ag2 mean_distance];
    end
end
[~,col] = min(record(:,3));
angle = zeros(19,1);
angle(4) = record(col,2);
angle(16) = record(col,1);
mesh = transform_angle(mesh, angle); % adjust MCP posture & update mesh
toc % 5min for processing

tic
record = [];
for ag3 = -0.3:0.0167:0.3
        angle = zeros(19,1);
        angle(4) = ag3;
        tr_mesh = transform_angle(mesh, angle);
        mean_distance = mean_dist_tester(tr_mesh, points, FRP_dorsal_segment, assignment_new);
        record = [record; ag3 mean_distance];
end
toc
[~,col] = min(record(:,2));
angle = zeros(19,1);
angle(4) = record(col,1);
mesh = transform_angle(mesh, angle); % adjust MCP posture & update mesh

% D3 MCP
FRP_dorsal_segment = 112;
record = [];
tic
for ag1 = -0.25:0.05:0.25
    for ag2 = 0:0.05:1
        angle = zeros(19,1);
        angle(7) = ag2;
        angle(17) = ag1;
        tr_mesh = transform_angle(mesh, angle);
        mean_distance = mean_dist_tester(tr_mesh, points, FRP_dorsal_segment, assignment_new);
        record = [record; ag1 ag2 mean_distance];
    end
end 
[~,col] = min(record(:,3));
Angle_opt.D3_MCP.AdAb = record(col,1); 
Angle_opt.D3_MCP.FxEt = record(col,2);
angle = zeros(19,1);
angle(7) = record(col,2);
angle(17) = record(col,1);
mesh = transform_angle(mesh, angle); % adjust MCP posture & update mesh
toc % 5min for processing

tic
record = [];
for ag3 = -0.3:0.0167:0.3
        angle = zeros(19,1);
        angle(7) = ag3;
        tr_mesh = transform_angle(mesh, angle);
        mean_distance = mean_dist_tester(tr_mesh, points, FRP_dorsal_segment, assignment_new);
        record = [record; ag3 mean_distance];
end
toc
[~,col] = min(record(:,2));
angle = zeros(19,1);
angle(7) = record(col,1);
mesh = transform_angle(mesh, angle); % adjust MCP posture & update mesh
Angle_opt.D3_MCP.FxEt2 = record(col,2);

% D4 MCP
FRP_dorsal_segment = 115;
record = [];
 tic
for ag1 = -0.25:0.05:0.25
    for ag2 = 0:0.05:1
        angle = zeros(19,1);
        angle(10) = ag2;
        angle(18) = ag1;
        tr_mesh = transform_angle(mesh, angle);
        mean_distance = mean_dist_tester(tr_mesh, points, FRP_dorsal_segment, assignment_new);
        record = [record; ag1 ag2 mean_distance];
    end
end 
 toc
[~,col] = min(record(:,3));
Angle_opt.D4_MCP.AdAb = record(col,1); 
Angle_opt.D4_MCP.FxEt = record(col,2);
angle = zeros(19,1);
angle(10) = record(col,2);
angle(18) = record(col,1);
mesh = transform_angle(mesh, angle); % adjust MCP posture & update mesh
% 5min for processing
tic
record = [];
for ag3 = -0.3:0.0167:0.3
        angle = zeros(19,1);
        angle(10) = ag3;
        tr_mesh = transform_angle(mesh, angle);
        mean_distance = mean_dist_tester(tr_mesh, points, FRP_dorsal_segment, assignment_new);
        record = [record; ag3 mean_distance];
end
toc
[~,col] = min(record(:,2));
angle = zeros(19,1);
angle(10) = record(col,1);
mesh = transform_angle(mesh, angle); % adjust MCP posture & update mesh
Angle_opt.D4_MCP.FxEt2 = record(col,2);

% D5 MCP 
FRP_dorsal_segment = 118;
record = [];
% tic
for ag1 = -0.25:0.05:0.25
    for ag2 = 0:0.05:1
        angle = zeros(19,1);
        angle(13) = ag2;
        angle(19) = ag1;
        tr_mesh = transform_angle(mesh, angle);
        mean_distance = mean_dist_tester(tr_mesh, points, FRP_dorsal_segment, assignment_new);
        record = [record; ag1 ag2 mean_distance];
    end
end 
toc
[~,col] = min(record(:,3));
Angle_opt.D5_MCP.AdAb = record(col,1); 
Angle_opt.D5_MCP.FxEt = record(col,2);
angle = zeros(19,1);
angle(13) = record(col,2);
angle(19) = record(col,1);
mesh = transform_angle(mesh, angle); % adjust MCP posture & update mesh
% 5min for processing
tic
record = [];
for ag3 = -0.3:0.0167:0.3
        angle = zeros(19,1);
        angle(13) = ag3;
        tr_mesh = transform_angle(mesh, angle);
        mean_distance = mean_dist_tester(tr_mesh, points, FRP_dorsal_segment, assignment_new);
        record = [record; ag3 mean_distance];
end
toc
[~,col] = min(record(:,2));
angle = zeros(19,1);
angle(13) = record(col,1);
mesh = transform_angle(mesh, angle); % adjust MCP posture & update mesh
Angle_opt.D5_MCP.FxEt2 = record(col,2);

% load('Angle_opt.mat');
% save angle
angle = zeros(19,1);
angle(17:19,1) = [%Angle_opt.D2_MCP.AdAb; ...
                  Angle_opt.D3_MCP.AdAb; Angle_opt.D4_MCP.AdAb; Angle_opt.D5_MCP.AdAb];
%angle(4)  = Angle_opt.D2_MCP.FxEt + Angle_opt.D2_MCP.FxEt2;
angle(7)  = Angle_opt.D3_MCP.FxEt + Angle_opt.D3_MCP.FxEt2;
angle(10) = Angle_opt.D4_MCP.FxEt + Angle_opt.D4_MCP.FxEt2;
angle(13) = Angle_opt.D5_MCP.FxEt + Angle_opt.D5_MCP.FxEt2;
save Angle_opt.mat angle Angle_opt
save mesh_HY_pos2.mat mesh % template posture align for HY_pos2.ply 


%% things to do - compare the better performance for MCP segment registration
% segment index of dorsal segment is better than whole part segment for pair generation 
% and fitting?

% % D4 MCP
% FRP_dorsal_segment = 15;
% record = [];
%  tic
% for ag1 = -0.25:0.05:0.25
%     for ag2 = 0:0.05:1
%         angle = zeros(19,1);
%         angle(10) = ag2;
%         angle(18) = ag1;
%         tr_mesh = transform_angle(mesh, angle);
%         mean_distance = mean_dist_tester(tr_mesh, points, FRP_dorsal_segment, mesh.assignment);
%         record = [record; ag1 ag2 mean_distance];
%     end
% end 
% % toc
% [~,col] = min(record(:,3));
% Angle_opt2.D4_MCP.AdAb = record(col,1); 
% Angle_opt2.D4_MCP.FxEt = record(col,2);
% 
% % D5 MCP 
% FRP_dorsal_segment = 18;
% record = [];
% % tic
% for ag1 = -0.25:0.05:0.25
%     for ag2 = 0:0.05:1
%         angle = zeros(19,1);
%         angle(13) = ag2;
%         angle(19) = ag1;
%         tr_mesh = transform_angle(mesh, angle);
%         mean_distance = mean_dist_tester(tr_mesh, points, FRP_dorsal_segment, mesh.assignment);
%         record = [record; ag1 ag2 mean_distance];
%     end
% end 
% toc
% [~,col] = min(record(:,3));
% Angle_opt2.D5_MCP.AdAb = record(col,1); 
% Angle_opt2.D5_MCP.FxEt = record(col,2);

%% D2 PIP & DIP fitting
% FRP_dorsal_segment = [6 109 112 115 118];

%mesh_BackUp = mesh;

% D2 PIP
FRP_dorsal_segment = 110;
tic
record = [];
for ag3 = 0:0.0167:1.5
        angle = zeros(19,1);
        angle(5) = ag3;
        tr_mesh = transform_angle(mesh, angle);
        mean_distance = mean_dist_tester(tr_mesh, points, FRP_dorsal_segment, assignment_new);
        record = [record; ag3 mean_distance];
end
toc
[~,col] = min(record(:,2));
angle = zeros(19,1);
angle(5) = record(col,1);
Angle_opt.D2_PIP.FxEt = record(col,1);
mesh = transform_angle(mesh, angle); % adjust MCP posture & update mesh

% D2 DIP
FRP_dorsal_segment = 111;
tic
record = [];
for ag3 = 0:0.0167:1.5
        angle = zeros(19,1);
        angle(6) = ag3;
        tr_mesh = transform_angle(mesh, angle);
        mean_distance = mean_dist_tester(tr_mesh, points, FRP_dorsal_segment, assignment_new);
        record = [record; ag3 mean_distance];
end
toc
[~,col] = min(record(:,2));
angle = zeros(19,1);
angle(6) = record(col,1);
Angle_opt.D2_DIP.FxEt = record(col,1);
mesh = transform_angle(mesh, angle); % adjust MCP posture & update mesh

%% D3 PIP & DIP fitting
% FRP_dorsal_segment = [6 109 112 115 118];

%mesh_BackUp = mesh;

% D3 PIP
FRP_dorsal_segment = 113;
tic
record = [];
for ag3 = 0:0.0167:1.5
        angle = zeros(19,1);
        angle(8) = ag3;
        tr_mesh = transform_angle(mesh, angle);
        mean_distance = mean_dist_tester(tr_mesh, points, FRP_dorsal_segment, assignment_new);
        record = [record; ag3 mean_distance];
end
toc
[~,col] = min(record(:,2));
angle = zeros(19,1);
angle(8) = record(col,1);
Angle_opt.D3_PIP.FxEt = record(col,1);
mesh = transform_angle(mesh, angle); % adjust MCP posture & update mesh

% D3 DIP
FRP_dorsal_segment = 114;
tic
record = [];
for ag3 = 0:0.0167:1.5
        angle = zeros(19,1);
        angle(9) = ag3;
        tr_mesh = transform_angle(mesh, angle);
        mean_distance = mean_dist_tester(tr_mesh, points, FRP_dorsal_segment, assignment_new);
        record = [record; ag3 mean_distance];
end
toc
[~,col] = min(record(:,2));
angle = zeros(19,1);
angle(9) = record(col,1);
Angle_opt.D3_DIP.FxEt = record(col,1);
mesh = transform_angle(mesh, angle); % adjust MCP posture & update mesh

%% D4 PIP & DIP fitting
% FRP_dorsal_segment = [6 109 112 115 118];

%mesh_BackUp = mesh;

% D4 PIP
FRP_dorsal_segment = 116;
tic
record = [];
for ag3 = 0:0.0167:1.5
        angle = zeros(19,1);
        angle(11) = ag3;
        tr_mesh = transform_angle(mesh, angle);
        mean_distance = mean_dist_tester(tr_mesh, points, FRP_dorsal_segment, assignment_new);
        record = [record; ag3 mean_distance];
end
toc
[~,col] = min(record(:,2));
angle = zeros(19,1);
angle(11) = record(col,1);
Angle_opt.D4_PIP.FxEt = record(col,1);
mesh = transform_angle(mesh, angle); % adjust MCP posture & update mesh

% D4 DIP
FRP_dorsal_segment = 117;
tic
record = [];
for ag3 = 0:0.0167:1.5
        angle = zeros(19,1);
        angle(12) = ag3;
        tr_mesh = transform_angle(mesh, angle);
        mean_distance = mean_dist_tester(tr_mesh, points, FRP_dorsal_segment, assignment_new);
        record = [record; ag3 mean_distance];
end
toc
[~,col] = min(record(:,2));
angle = zeros(19,1);
angle(12) = record(col,1);
Angle_opt.D4_DIP.FxEt = record(col,1);
mesh = transform_angle(mesh, angle); % adjust MCP posture & update mesh

%% D45 PIP & DIP fitting
% FRP_dorsal_segment = [6 109 112 115 118];

%mesh_BackUp = mesh;

% D4 PIP
FRP_dorsal_segment = 119;
tic
record = [];
for ag3 = 0:0.0167:1.5
        angle = zeros(19,1);
        angle(14) = ag3;
        tr_mesh = transform_angle(mesh, angle);
        mean_distance = mean_dist_tester(tr_mesh, points, FRP_dorsal_segment, assignment_new);
        record = [record; ag3 mean_distance];
end
toc
[~,col] = min(record(:,2));
angle = zeros(19,1);
angle(14) = record(col,1);
Angle_opt.D5_PIP.FxEt = record(col,1);
mesh = transform_angle(mesh, angle); % adjust MCP posture & update mesh

% D4 DIP
FRP_dorsal_segment = 120;
tic
record = [];
for ag3 = 0:0.0167:1.5
        angle = zeros(19,1);
        angle(15) = ag3;
        tr_mesh = transform_angle(mesh, angle);
        mean_distance = mean_dist_tester(tr_mesh, points, FRP_dorsal_segment, assignment_new);
        record = [record; ag3 mean_distance];
end
toc
[~,col] = min(record(:,2));
angle = zeros(19,1);
angle(15) = record(col,1);
Angle_opt.D5_DIP.FxEt = record(col,1);
mesh = transform_angle(mesh, angle); % adjust MCP posture & update mesh

angle = zeros(19,1);
angle(17:19,1) = [%Angle_opt.D2_MCP.AdAb; ...
                  Angle_opt.D3_MCP.AdAb; Angle_opt.D4_MCP.AdAb; Angle_opt.D5_MCP.AdAb];
%angle(4)  = Angle_opt.D2_MCP.FxEt + Angle_opt.D2_MCP.FxEt2;
angle(7)  = Angle_opt.D3_MCP.FxEt + Angle_opt.D3_MCP.FxEt2;
angle(10) = Angle_opt.D4_MCP.FxEt + Angle_opt.D4_MCP.FxEt2;
angle(13) = Angle_opt.D5_MCP.FxEt + Angle_opt.D5_MCP.FxEt2;
save Angle_opt.mat angle Angle_opt
save mesh_HY_pos2_2.mat mesh % template posture align for HY_pos2.ply 


%% Things to do (thumb registration)

%% ICP registration

targetV = points.vertices;
sourceV = mesh.vertices;
targetF = points.faces;
sourceF = mesh.faces;
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
%scatter3(centers_c(:,1),centers_c(:,2),centers_c(:,3),'o','MarkerEdgeColor',[255/255, 0/255, 0/255]);
hold off

%% save mat file
V_HY_pos2 = sourceV;
save HY_pos2_vertices.mat V_HY_pos2

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










