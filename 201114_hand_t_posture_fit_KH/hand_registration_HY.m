clc
clear all

%% register library - PC Home
addpath(genpath('../external'));
addpath('D:\GitHub\Hand_deformation_model\functions');
addpath('Data');
addpath('D:\GitHub\Hand_deformation_model\external\registration');

%% register library - PC lab
addpath(genpath('../external'));
addpath('F:\[GitHub]\Hand_deformation_model\functions');
addpath('F:\[GitHub]\Hand_deformation_model\data_SW');
addpath('F:\[GitHub]\Hand_deformation_model\data');
addpath('F:\[GitHub]\Hand_deformation_model\external\registration');

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
%mesh = transformed;

%% parameter for finger root (MCP) registration 

% ==== start here
% things to do 
% MCP D2 - rotation angle combination set generation, fitting, dist_arry
% bath


FRP_segment = [6 9 12 15 18];
FRP_dorsal_segment = [6 109 112 115 118];
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
Recorder = [];

for j = 1:3

vertices = mesh.vertices;
faces = mesh.faces;
normals = per_vertex_normals(vertices, faces);
keep = ismember(mesh.assignment, FRP_segment(j));
%keep = ismember(assignment_new, FRP_dorsal_segment(j));
[vertices, faces] = filter_vertices(vertices, faces, keep);
normals = normals(keep, :);

%pairs = compute_correspondences_modi_MCP(vertices, normals, points.vertices, points.normals, 25);
pairs = compute_correspondences_new(vertices, normals, points.vertices, points.normals, 20, cos(45*pi/180));
mean_distance = dist_sum(vertices, points.vertices, pairs)/size(pairs,1)

%pairs = correspondences_rigid(vertices, normals, points.vertices, points.normals, 20, cos(45*pi/180));
%pairs = correspondences_dorsal(vertices_dorsal, normals_dorsal, points.vertices, points.normals, 20, cos(45*pi/180));

transform = eye(4);

figure(2)
    view(view_angle);
    for i = 1 : 10
        delete(h3);
        delete(h4);
        delta = compute_transformation(vertices, points.vertices, points.normals, pairs);
        delta = constraint_transformation(delta, mesh.spheres{1,FRP_cor(j)}.center);
                
        transform = delta * transform;
        vertices = apply_matrix(delta, vertices);
        normals = apply_matrix(delta, normals, 0);
        
        mean_distance = [mean_distance; dist_sum(vertices, points.vertices, pairs)/size(pairs,1)]
        %Recorder = [Recorder; j i mean_distance 20 cos(30*pi/180)];
        
%       pairs = compute_correspondences_modi_MCP(vertices, normals, points.vertices, points.normals, 20, cos(45*pi/180));
        pairs = compute_correspondences_new(vertices, normals, points.vertices, points.normals, 20, cos(30*pi/180));
%       pairs = correspondences_dorsal(vertices, normals, points.vertices, points.normals, 20, cos(45*pi/180));

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
vi_Dx = mesh.vertices(keep,:);
vi_Dx = apply_matrix(transform, vi_Dx, 1);

mesh.vertices(keep,:) = vi_Dx;
mesh.normals = per_vertex_normals(mesh.vertices, mesh.faces);

    for i = [FRP_cor_tr{j}]
        mesh.spheres{1,i}.center = apply_matrix(transform, mesh.spheres{1,i}.center,1);
        mesh.centers(i,:) = mesh.spheres{1,i}.center;
    end

end

% optimal problem to find proper threshold distance & angle

for j = 4

vertices = mesh.vertices;
faces = mesh.faces;
normals = per_vertex_normals(vertices, faces);
keep = ismember(mesh.assignment, FRP_segment(j));
%keep = ismember(assignment_new, FRP_dorsal_segment(j));
[vertices, faces] = filter_vertices(vertices, faces, keep);
normals = normals(keep, :);

%pairs = compute_correspondences_modi_MCP(vertices, normals, points.vertices, points.normals, 25);
pairs = compute_correspondences_new(vertices, normals, points.vertices, points.normals, 20, cos(45*pi/180));
mean_distance = dist_sum(vertices, points.vertices, pairs)/size(pairs,1)

%pairs = correspondences_rigid(vertices, normals, points.vertices, points.normals, 20, cos(45*pi/180));
%pairs = correspondences_dorsal(vertices_dorsal, normals_dorsal, points.vertices, points.normals, 20, cos(45*pi/180));

transform = eye(4);

figure(2)
    view(-37,19);
    for i = 1 : 10
        delete(h3);
        delete(h4);
        delta = compute_transformation(vertices, points.vertices, points.normals, pairs);
        delta = constraint_transformation(delta, mesh.spheres{1,FRP_cor(j)}.center);
                
        transform = delta * transform;
        vertices = apply_matrix(delta, vertices);
        normals = apply_matrix(delta, normals, 0);
        
        mean_distance = [mean_distance; dist_sum(vertices, points.vertices, pairs)/size(pairs,1)]
        
%       pairs = compute_correspondences_modi_MCP(vertices, normals, points.vertices, points.normals, 20, cos(45*pi/180));
       pairs = compute_correspondences_new(vertices, normals, points.vertices, points.normals, 25, cos(30*pi/180));
%        pairs = correspondences_dorsal(vertices, normals, points.vertices, points.normals, 20, cos(45*pi/180));

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
vi_Dx = mesh.vertices(keep,:);
vi_Dx = apply_matrix(transform, vi_Dx, 1);

mesh.vertices(keep,:) = vi_Dx;
mesh.normals = per_vertex_normals(mesh.vertices, mesh.faces);

    for i = [FRP_cor_tr{j}]
        mesh.spheres{1,i}.center = apply_matrix(transform, mesh.spheres{1,i}.center,1);
        mesh.centers(i,:) = mesh.spheres{1,i}.center;
    end

end

for j = 5

vertices = mesh.vertices;
faces = mesh.faces;
normals = per_vertex_normals(vertices, faces);
keep = ismember(mesh.assignment, FRP_segment(j));
%keep = ismember(assignment_new, FRP_dorsal_segment(j));
[vertices, faces] = filter_vertices(vertices, faces, keep);
normals = normals(keep, :);

%pairs = compute_correspondences_modi_MCP(vertices, normals, points.vertices, points.normals, 25);
pairs = compute_correspondences_new(vertices, normals, points.vertices, points.normals, 20, cos(40*pi/180));
mean_distance = dist_sum(vertices, points.vertices, pairs)/size(pairs,1)

%pairs = correspondences_rigid(vertices, normals, points.vertices, points.normals, 20, cos(45*pi/180));
%pairs = correspondences_dorsal(vertices_dorsal, normals_dorsal, points.vertices, points.normals, 20, cos(45*pi/180));

transform = eye(4);

figure(2)
    view(-37,19);
    for i = 1 : 10
        delete(h3);
        delete(h4);
        delta = compute_transformation(vertices, points.vertices, points.normals, pairs);
        delta = constraint_transformation(delta, mesh.spheres{1,FRP_cor(j)}.center);
                
        transform = delta * transform;
        vertices = apply_matrix(delta, vertices);
        normals = apply_matrix(delta, normals, 0);
        
%       pairs = compute_correspondences_modi_MCP(vertices, normals, points.vertices, points.normals, 20, cos(45*pi/180));
       pairs = compute_correspondences_new(vertices, normals, points.vertices, points.normals, 25, cos(35*pi/180));
       mean_distance = [mean_distance; dist_sum(vertices, points.vertices, pairs)/size(pairs,1)]
       
%        pairs = correspondences_dorsal(vertices, normals, points.vertices, points.normals, 20, cos(45*pi/180));

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
vi_Dx = mesh.vertices(keep,:);
vi_Dx = apply_matrix(transform, vi_Dx, 1);

mesh.vertices(keep,:) = vi_Dx;
mesh.normals = per_vertex_normals(mesh.vertices, mesh.faces);

    for i = [FRP_cor_tr{j}]
        mesh.spheres{1,i}.center = apply_matrix(transform, mesh.spheres{1,i}.center,1);
        mesh.centers(i,:) = mesh.spheres{1,i}.center;
    end

end

figure()
axis equal
axis off
hold on
view(view_angle);
scatter3(points.vertices(:,1),points.vertices(:,2),points.vertices(:,3),'.', 'MarkerEdgeColor',[180/255, 180/255, 180/255]);
scatter3(mesh.vertices(:,1),mesh.vertices(:,2),mesh.vertices(:,3),'.', 'MarkerEdgeColor',[190/255, 240/255, 251/255]);
%scatter3(mesh.centers(:,1),mesh.centers(:,2),mesh.centers(:,3),'o','MarkerEdgeColor',[255/255, 0/255, 0/255]);
for i = 1:22
    scatter3(mesh.spheres{1,i}.center(1),mesh.spheres{1,i}.center(2),mesh.spheres{1,i}.center(3),'o','MarkerEdgeColor',[255/255, 0/255, 0/255]);
end 
hold off

% ====================== start here

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
view(view_angle);
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
view(view_angle);
scatter3(points.vertices(:,1),points.vertices(:,2),points.vertices(:,3),'.', 'MarkerEdgeColor',[180/255, 180/255, 180/255]);
scatter3(hand_template.vertices(:,1),hand_template.vertices(:,2),hand_template.vertices(:,3),'.', 'MarkerEdgeColor',[190/255, 240/255, 251/255]);
% scatter3(hand_template.centers(:,1),hand_template.centers(:,2),hand_template.centers(:,3),'o','MarkerEdgeColor',[255/255, 0/255, 0/255]);
hold off

vertices_c = hand_template.vertices;
normals = hand_template.normals;

%% things to do 
% separate PIP - DIP registration
% better to register by fingers...
% segment length check
% restrict finger motion only for flexion / extension

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
%pairs = compute_correspondences_modi_MCP(vertices, normals, points.vertices, points.normals);
pairs = compute_correspondences_new(vertices, normals, points.vertices, points.normals,20,cos(60*pi/180));

transform = eye(4);

figure(2)
    view(view_angle);
    for i = 1 : 10
        delete(h3);
        delete(h4);
        delta = compute_transformation(vertices, points.vertices, points.normals, pairs);
        delta = constraint_transformation(delta, centers_c(P_cor(j),:));
        transform = delta * transform;
        vertices = apply_matrix(delta, vertices);
        normals = apply_matrix(delta, normals, 0);
%        pairs = compute_correspondences_modi_MCP(vertices, normals, points.vertices, points.normals,20,cos(60*pi/180));
         pairs = compute_correspondences_new(vertices, normals, points.vertices, points.normals,25);
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
V_P1 = sourceV;
save P1_vertices.mat V_P1

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










