clc
clear all

%% register library - PC Home
addpath(genpath('../external'));
addpath('D:\GitHub\Hand_deformation_model\functions');
addpath('Data');
addpath('D:\GitHub\Hand_deformation_model\data_SW');
addpath('D:\GitHub\Hand_deformation_model\201026_hand_t_update');
addpath('D:\GitHub\Hand_deformation_model\external\registration');
addpath 'D:\GitHub\Hand_deformation_model\200916_registration_hand_SW'
addpath('D:\GitHub\Hand_deformation_model\data');

clc
clear all
addpath(genpath('../external'));
addpath('/Users/hayoungjung/Documents/GitHub/Hand_deformation_model/data');
addpath('/Users/hayoungjung/Documents/GitHub/Hand_deformation_model/data_SW');
addpath('/Users/hayoungjung/Documents/GitHub/Hand_deformation_model/functions');
addpath('/Users/hayoungjung/Documents/GitHub/Hand_deformation_model/external/registration');
addpath('/Users/hayoungjung/Documents/GitHub/Hand_deformation_model/200916_registration_hand_SW');
format shortG

%% Load data
global assignment_new
load('hy_mesh_n5.mat'); %template
load('assignment_new.mat');
[points.vertices, points.faces, points.FB, points.H] = function_loading_ply_file('JB_pos3.ply'); % target scan
points.normals = per_vertex_normals(points.vertices, points.faces);
LMs_PLM = function_get_LM_from_iges('JB_pos3_PLM.igs'); % LM for scan
LMt_PLM = function_get_LM_from_iges('LMt.igs'); % LM for template

% %% search template LM index
% LMt = function_get_LM_from_iges('template_LMs.igs'); % template landmark by hand
% LMt(46:49,:) = function_get_LM_from_iges('template_tip_scale_LMs.igs'); %template landmark tip
% LMs = function_get_LM_from_iges('JB_pos2_scale_LM.igs'); % scan landmark by hand 
% LMt_Idx = zeros(size(LMt,1),1);
% m = size(mesh.vertices, 1);
% for i = 1:size(LMt,1)
% delta = mesh.vertices - repmat(LMt(i, :), m, 1);
% distances = sum(delta .^ 2, 2);
% [~, j] = min(distances);
% LMt_Idx(i,:) = j; % template landmark index detection
% end
% for i = 1:size(LMt,1)
% LMt(i,:) = mesh.vertices(LMt_Idx(i),:); % template landmark update
% end
% clear m delta distances i j 

%% palm fitting

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

% save hy_mesh_n5_palm_fitted.mat mesh %template
% mesh.spheres{1,22}.center

%%
centers = zeros(30,3);
for i = 1:30
centers(i,:) = mesh.spheres{1,i}.center;
end

A = centers(1:22,:); % template's CoR
B = []; %import data from excel sheet

figure()
hold on
axis equal
axis off
plot3(A(:,1),A(:,2),A(:,3),'k*')
plot3(A(1:4,1),A(1:4,2),A(1:4,3),'-b')
plot3(A(5:8,1),A(5:8,2),A(5:8,3),'-b')
plot3(A(9:12,1),A(9:12,2),A(9:12,3),'-b')
plot3(A(13:16,1),A(13:16,2),A(13:16,3),'-b')
plot3(A([17:20 22],1),A([17:20 22],2),A([17:20 22],3),'-b')
plot3(A([4 22 8],1),A([4 22 8],2),A([4 22 8],3),'-b')
plot3(A([12 22 16],1),A([12 22 16],2),A([12 22 16],3),'-b')
plot3(B(:,1),B(:,2),B(:,3),'r*');
plot3(B(1:4,1),B(1:4,2),B(1:4,3),'-r');
plot3(B(5:8,1),B(5:8,2),B(5:8,3),'-r');
plot3(B(9:12,1),B(9:12,2),B(9:12,3),'-r');
plot3(B(13:16,1),B(13:16,2),B(13:16,3),'-r');
plot3(B([4 22 8],1),B([4 22 8],2),B([4 22 8],3),'-r')
plot3(B([12 22 16],1),B([12 22 16],2),B([12 22 16],3),'-r')
hold off

%% scale
% compare_factor = compare_factor_cal(A,B);
% A_tr = segment_scale_simple(A, compare_factor);
% A = A_tr;

[transformed] = segment_scale_fingers_new2(mesh, A, B);
mesh = transformed;
clear transformed


%% calculate optimal rotation angle for MCP & deformation
% FRP_dorsal_segment = [6 109 112 115 118];

% function X = (segment, mesh.v, points.v) ==> Y = (angle X)
% function X2 = (angle1, angle2) ==> Y = mean distance
% function find_opt_rotation
tic

%% D2 Register

tic
angle = zeros(19,1);
% D2 MCP fit
FRP_dorsal_segment = 109;
record = zeros(250,3);
record(:,3) = 1000;
w = 1;
for ag1 = -0.25:0.05:0.25 % size = 11
    for ag2 = 0:0.05:1 % size = 21
        angle = zeros(19,1);
        angle(4) = ag2;
        angle(16) = ag1;
        tr_mesh = transform_angle(mesh, angle);
        mean_distance = mean_dist_tester(tr_mesh, points, FRP_dorsal_segment, assignment_new);
        record(w,:) = [ag1 ag2 mean_distance];
        w = w+1;
    end
end
[~,col] = min(record(:,3));
%Angle_opt.D2_MCP.AdAb = record(col,1); 
%Angle_opt.D2_MCP.FxEt = record(col,2);
angle = zeros(19,1);
angle(4) = record(col,2);
angle(16) = record(col,1);
mesh = transform_angle(mesh, angle); % adjust MCP posture & update mesh
visualization
toc

%% 
tic
record = [];
for ag3 = -0.3:0.0167:0.3 % size = 36
        angle = zeros(19,1);
        angle(4) = ag3;
        tr_mesh = transform_angle(mesh, angle);
        mean_distance = mean_dist_tester(tr_mesh, points, FRP_dorsal_segment, assignment_new);
        record = [record; ag3 mean_distance];
end
[~,col] = min(record(:,2));
angle = zeros(19,1);
angle(4) = record(col,1);
mesh = transform_angle(mesh, angle); % adjust MCP posture & update mesh
visualization
%Angle_opt.D2_MCP.FxEt2 = record(col,2);
toc
%%
% D2 PIP fit
FRP_dorsal_segment = 110;
record = [];
for ag3 = 0:0.03:2 % size = 67
        angle = zeros(19,1);
        angle(5) = ag3;
        tr_mesh = transform_angle(mesh, angle);
        mean_distance = mean_dist_tester(tr_mesh, points, FRP_dorsal_segment, assignment_new);
        record = [record; ag3 mean_distance];
end
[~,col] = min(record(:,2));
angle = zeros(19,1);
angle(5) = record(col,1);
Angle_opt.D2_PIP.FxEt = record(col,1);
mesh = transform_angle(mesh, angle); % adjust MCP posture & update mesh

% D2 MCP supination/pronation fit
record = [];
for ag3 = -0.28:0.07:0.28 % size = 9
        angle_supr = zeros(4,1);
        angle_supr(1) = ag3;
        tr_mesh = transform_sup_pro_angle(mesh, angle_supr);
        mean_distance = mean_dist_tester(tr_mesh, points, FRP_dorsal_segment, assignment_new);
        record = [record; ag3 mean_distance];
end
[~,col] = min(record(:,2));
angle_supr = zeros(4,1);
angle_supr(1) = record(col,1);
Angle_opt.D2_PIP.SpPr = record(col,1);
mesh = transform_sup_pro_angle(mesh, angle_supr); % adjust MCP posture & update mesh

% D2 DIP
FRP_dorsal_segment = 111;
record = [];
for ag3 = -0.25:0.03:1.25 % size = 26
        angle = zeros(19,1);
        angle(6) = ag3;
        tr_mesh = transform_angle(mesh, angle);
        mean_distance = mean_dist_tester(tr_mesh, points, FRP_dorsal_segment, assignment_new);
        record = [record; ag3 mean_distance];
end
[~,col] = min(record(:,2));
angle = zeros(19,1);
angle(6) = record(col,1);
Angle_opt.D2_DIP.FxEt = record(col,1);
mesh = transform_angle(mesh, angle); % adjust MCP posture & update mesh
visualization
toc

%% 
% D3 Register
tic

% D3 MCP fit
FRP_dorsal_segment = 112;
record = [];
for ag1 = -0.25:0.05:0.25 % size = 11
    for ag2 = 0:0.05:1 % size = 21
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
visualization

toc 
%%
record = [];
for ag3 = -0.3:0.0167:0.3 % size = 36
        angle = zeros(19,1);
        angle(7) = ag3;
        tr_mesh = transform_angle(mesh, angle);
        mean_distance = mean_dist_tester(tr_mesh, points, FRP_dorsal_segment, assignment_new);
        record = [record; ag3 mean_distance];
end
[~,col] = min(record(:,2));
angle = zeros(19,1);
angle(7) = record(col,1);
mesh = transform_angle(mesh, angle); % adjust MCP posture & update mesh
%Angle_opt.D3_MCP.FxEt2 = record(col,2);

%% D3 PIP fit
FRP_dorsal_segment = 113;
record = [];
for ag3 = 0:0.03:2 % size = 67
        angle = zeros(19,1);
        angle(8) = ag3;
        tr_mesh = transform_angle(mesh, angle);
        mean_distance = mean_dist_tester(tr_mesh, points, FRP_dorsal_segment, assignment_new);
        record = [record; ag3 mean_distance];
end
[~,col] = min(record(:,2));
angle = zeros(19,1);
angle(8) = record(col,1);
%Angle_opt.D3_PIP.FxEt = record(col,1);
mesh = transform_angle(mesh, angle); % adjust MCP posture & update mesh
visualization

%%
record = [];
for ag3 = -0.28:0.07:0.28 % size = 9
        angle_supr = zeros(4,1);
        angle_supr(2) = ag3;
        tr_mesh = transform_sup_pro_angle(mesh, angle_supr);
        mean_distance = mean_dist_tester(tr_mesh, points, FRP_dorsal_segment, assignment_new);
        record = [record; ag3 mean_distance];
end
[~,col] = min(record(:,2));
angle_supr = zeros(4,1);
angle_supr(2) = record(col,1);
Angle_opt.D3_PIP.SpPr = record(col,1);
mesh = transform_sup_pro_angle(mesh, angle_supr); % adjust MCP posture & update mesh

%% D3 DIP
tic
FRP_dorsal_segment = 114;
record = [];
for ag3 = -0.25:0.03:1.25 % size = 26
        angle = zeros(19,1);
        angle(9) = ag3;
        tr_mesh = transform_angle(mesh, angle);
        mean_distance = mean_dist_tester(tr_mesh, points, FRP_dorsal_segment, assignment_new);
        record = [record; ag3 mean_distance];
end
[~,col] = min(record(:,2));
angle = zeros(19,1);
angle(9) = record(col,1);
Angle_opt.D3_DIP.FxEt = record(col,1);
mesh = transform_angle(mesh, angle); % adjust MCP posture & update mesh
visualization
toc
%%
% D4 register

tic

% D4 MCP fit
FRP_dorsal_segment = 115;
record = [];
for ag1 = -0.25:0.05:0.25 % size = 11
    for ag2 = 0:0.05:1 % size = 21
        angle = zeros(19,1);
        angle(10) = ag2;
        angle(18) = ag1;
        tr_mesh = transform_angle(mesh, angle);
        mean_distance = mean_dist_tester(tr_mesh, points, FRP_dorsal_segment, assignment_new);
        record = [record; ag1 ag2 mean_distance];
    end
end
[~,col] = min(record(:,3));
Angle_opt.D4_MCP.AdAb = record(col,1); 
Angle_opt.D4_MCP.FxEt = record(col,2);
angle = zeros(19,1);
angle(10) = record(col,2);
angle(18) = record(col,1);
mesh = transform_angle(mesh, angle); % adjust MCP posture & update mesh
visualization

%%
FRP_dorsal_segment = 115;
record = [];
for ag3 = -0.3:0.0167:0.3 % size = 36
        angle = zeros(19,1);
        angle(10) = ag3;
        tr_mesh = transform_angle(mesh, angle);
        mean_distance = mean_dist_tester(tr_mesh, points, FRP_dorsal_segment, assignment_new);
        record = [record; ag3 mean_distance];
end
[~,col] = min(record(:,2));
angle = zeros(19,1);
angle(10) = record(col,1);
mesh = transform_angle(mesh, angle); % adjust MCP posture & update mesh
%Angle_opt.D4_MCP.FxEt2 = record(col,2);
visualization

FRP_dorsal_segment = 116;
record = [];
for ag3 = 0:0.03:2 % size = 67
        angle = zeros(19,1);
        angle(11) = ag3;
        tr_mesh = transform_angle(mesh, angle);
        mean_distance = mean_dist_tester(tr_mesh, points, FRP_dorsal_segment, assignment_new);
        record = [record; ag3 mean_distance];
end
[~,col] = min(record(:,2));
angle = zeros(19,1);
angle(11) = record(col,1);
%Angle_opt.D4_PIP.FxEt = record(col,1);
mesh = transform_angle(mesh, angle); % adjust MCP posture & update mesh
visualization

%%
record = [];
for ag3 = -0.28:0.07:0.28 % size = 9
        angle_supr = zeros(4,1);
        angle_supr(3) = ag3;
        tr_mesh = transform_sup_pro_angle(mesh, angle_supr);
        mean_distance = mean_dist_tester(tr_mesh, points, FRP_dorsal_segment, assignment_new);
        record = [record; ag3 mean_distance];
end
[~,col] = min(record(:,2));
angle_supr = zeros(4,1);
angle_supr(3) = record(col,1);
Angle_opt.D4_PIP.SpPr = record(col,1);
mesh = transform_sup_pro_angle(mesh, angle_supr); % adjust MCP posture & update mesh
%%
% D4 DIP
FRP_dorsal_segment = 117;
record = [];
for ag3 = -0.25:0.03:1.25 % size = 26
        angle = zeros(19,1);
        angle(12) = ag3;
        tr_mesh = transform_angle(mesh, angle);
        mean_distance = mean_dist_tester(tr_mesh, points, FRP_dorsal_segment, assignment_new);
        record = [record; ag3 mean_distance];
end
[~,col] = min(record(:,2));
angle = zeros(19,1);
angle(12) = record(col,1);
Angle_opt.D4_DIP.FxEt = record(col,1);
mesh = transform_angle(mesh, angle); % adjust MCP posture & update mesh

toc
%%
% D5 register

tic

% D5 MCP fit
FRP_dorsal_segment = 118;
record = [];
for ag1 = -0.25:0.05:0.25 % size = 11
    for ag2 = 0:0.05:1 % size = 21
        angle = zeros(19,1);
        angle(13) = ag2;
        angle(19) = ag1;
        tr_mesh = transform_angle(mesh, angle);
        mean_distance = mean_dist_tester(tr_mesh, points, FRP_dorsal_segment, assignment_new);
        record = [record; ag1 ag2 mean_distance];
    end
end
[~,col] = min(record(:,3));
Angle_opt.D5_MCP.AdAb = record(col,1); 
Angle_opt.D5_MCP.FxEt = record(col,2);
angle = zeros(19,1);
angle(13) = record(col,2);
angle(19) = record(col,1);
mesh = transform_angle(mesh, angle); % adjust MCP posture & update mesh

record = [];
for ag3 = -0.3:0.0167:0.3 % size = 36
        angle = zeros(19,1);
        angle(13) = ag3;
        tr_mesh = transform_angle(mesh, angle);
        mean_distance = mean_dist_tester(tr_mesh, points, FRP_dorsal_segment, assignment_new);
        record = [record; ag3 mean_distance];
end
[~,col] = min(record(:,2));
angle = zeros(19,1);
angle(13) = record(col,1);
mesh = transform_angle(mesh, angle); % adjust MCP posture & update mesh
Angle_opt.D5_MCP.FxEt2 = record(col,2);
visualization

%%
FRP_dorsal_segment = 119;
record = [];
for ag3 = 0:0.03:2 % size = 67
        angle = zeros(19,1);
        angle(14) = ag3;
        tr_mesh = transform_angle(mesh, angle);
        mean_distance = mean_dist_tester(tr_mesh, points, FRP_dorsal_segment, assignment_new);
        record = [record; ag3 mean_distance];
end
[~,col] = min(record(:,2));
angle = zeros(19,1);
angle(14) = record(col,1);
Angle_opt.D5_PIP.FxEt = record(col,1);
mesh = transform_angle(mesh, angle); % adjust MCP posture & update mesh

record = [];
for ag3 = -0.28:0.07:0.28 % size = 9
        angle_supr = zeros(4,1);
        angle_supr(4) = ag3;
        tr_mesh = transform_sup_pro_angle(mesh, angle_supr);
        mean_distance = mean_dist_tester(tr_mesh, points, FRP_dorsal_segment, assignment_new);
        record = [record; ag3 mean_distance];
end
[~,col] = min(record(:,2));
angle_supr = zeros(4,1);
angle_supr(4) = record(col,1);
Angle_opt.D5_PIP.SpPr = record(col,1);
mesh = transform_sup_pro_angle(mesh, angle_supr); % adjust MCP posture & update mesh
visualization

%%
% D5 DIP
FRP_dorsal_segment = 120;
record = [];
for ag3 = -0.25:0.03:1.25 % size = 26
        angle = zeros(19,1);
        angle(15) = ag3;
        tr_mesh = transform_angle(mesh, angle);
        mean_distance = mean_dist_tester(tr_mesh, points, FRP_dorsal_segment, assignment_new);
        record = [record; ag3 mean_distance];
end
[~,col] = min(record(:,2));
angle = zeros(19,1);
angle(15) = record(col,1);
Angle_opt.D5_DIP.FxEt = record(col,1);
mesh = transform_angle(mesh, angle); % adjust MCP posture & update mesh
visualization

toc

%%
angle = zeros(19,1);
angle(16:19,1) = [Angle_opt.D2_MCP.AdAb; ...
                  Angle_opt.D3_MCP.AdAb; Angle_opt.D4_MCP.AdAb; Angle_opt.D5_MCP.AdAb];
angle(4)  = Angle_opt.D2_MCP.FxEt + Angle_opt.D2_MCP.FxEt2;
angle(7)  = Angle_opt.D3_MCP.FxEt + Angle_opt.D3_MCP.FxEt2;
angle(10) = Angle_opt.D4_MCP.FxEt + Angle_opt.D4_MCP.FxEt2;
angle(13) = Angle_opt.D5_MCP.FxEt + Angle_opt.D5_MCP.FxEt2;
% save Angle_opt.mat angle Angle_opt
% save mesh_HY_pos2.mat mesh % template posture align for HY_pos2.ply
 save Angle_opt_HY_pos2.mat angle Angle_opt
 save mesh_HY_pos2.mat mesh % template posture align for MJ_pos6.ply 

% things to do - compare the better performance for MCP segment registration
% segment index of dorsal segment is better than whole part segment for pair generation 
% and fitting?

%% Things to do (thumb registration - temporal) 

tic

FRP_dorsal_segment = [6 107 108];
FRP_cor = [20 19 18];
FRP_digits{1} = [6:8];
FRP_cor_tr{1} = [17:19];
transform_order= [3 4 5];

axes = compute_bone_axes(mesh.spheres);
normals = per_vertex_normals(mesh.vertices, mesh.faces);

h3 = [];
h4 = [];

transforms = cell(1, 18);
for i = 1 : 18
    transforms{i} = eye(4);
end

for j = 1:3
vertices = mesh.vertices;
faces = mesh.faces;
normals = per_vertex_normals(vertices, faces);
keep = ismember(assignment_new, FRP_dorsal_segment(j));
[vertices, faces] = filter_vertices(vertices, faces, keep);
normals = normals(keep, :);
pairs = compute_correspondences_new(vertices, normals, points.vertices, points.normals, 20, cos(45*pi/180));

transform = eye(4);

figure()
    view(view_angle);
    for i = 1 : 15
        delete(h3);
        delete(h4);
        delta = compute_transformation(vertices, points.vertices, points.normals, pairs);
        delta = constraint_transformation(delta, mesh.spheres{1,FRP_cor(j)}.center); %centers_c(FRP_cor(j),:));
        transform = delta * transform;
        vertices = apply_matrix(delta, vertices);
        normals = apply_matrix(delta, normals, 0);
        
       pairs = compute_correspondences_new(vertices, normals, points.vertices, points.normals, 25, cos(45*pi/180));

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

transformed = skin_dualquat(mesh, transforms);
    for i = 1:18
         transformed.centers(i,:) = transformed.spheres{1,i}.center;
    end
mesh = transformed;

h3 = [];
h4 = [];

    transforms = cell(1, 18);
    for i = 1 : 18
        transforms{i} = eye(4);
    end

end 

toc

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
%V_HY_pos2 = sourceV;
%save HY_pos2_vertices.mat V_HY_pos2

JB_pos2_vertices = sourceV;
mesh.vertices = sourceV;
save JB_pos2_vertices.mat JB_pos2_vertices;


% save function 
% Header ����
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










