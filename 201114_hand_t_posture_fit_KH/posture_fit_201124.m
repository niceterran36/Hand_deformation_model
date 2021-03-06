clc
clear all
close all

%% register library

% window OS
addpath(genpath('../external'));
addpath('D:\GitHub\Hand_deformation_model\functions');
addpath('Data');
addpath('D:\GitHub\Hand_deformation_model\data_SW');
addpath('D:\GitHub\Hand_deformation_model\201026_hand_t_update');
addpath('D:\GitHub\Hand_deformation_model\external\registration');
addpath 'D:\GitHub\Hand_deformation_model\200916_registration_hand_SW'
addpath('D:\GitHub\Hand_deformation_model\data');

% Mac OS
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
load('JcorMx.mat');
%dirLM = dir('D:\GitHub\Hand_deformation_model\data_SW\*.igs');
%dir3D = dir('D:\GitHub\Hand_deformation_model\data_SW\*.ply');
dirLM = dir('/Users/hayoungjung/Documents/GitHub/Hand_deformation_model/data_SW/*.igs');
dir3D = dir('/Users/hayoungjung/Documents/GitHub/Hand_deformation_model/data_SW/*.ply');

% ZC_LMs_PLM = 73 ~ 81
% JW_3D_PLY  = 72 ~ 80

iii = 7;
% j = i + 26;
% k = j+1;

[points.vertices, points.faces, points.FB, points.H] = function_loading_ply_file(dir3D(iii+71).name); 
% target scan -- e.g.) 'JW_pos01.ply'
points.normals = per_vertex_normals(points.vertices, points.faces);

LMs_PLM = function_get_LM_from_iges(dirLM(iii+72).name); % LM for scan -- e.g.) 'DH_pos9_PLM.igs'
LMt_PLM = function_get_LM_from_iges('LMt.igs'); % LM for template
fprintf('%s is loaded \n',dir3D(iii+26).name);
fprintf('%s is loaded \n',dirLM(iii+27).name);
%sprintf('%s', dir3D(i).name)

B = JcorMx.ZC(-21+22*iii:22*iii,:);
% iii =1 1:22   -21+22*iii ~ 22*iii
% iii =2 23:44  -21+22*iii ~ 22*iii
% iii =3 45:66  -21+22*iii ~ 22*iii

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

fprintf('palm registration is completed\n');

% save hy_mesh_n5_palm_fitted.mat mesh %template
% mesh.spheres{1,22}.center

%%

centers = zeros(30,3);
for i = 1:30
centers(i,:) = mesh.spheres{1,i}.center;
end
A = centers(1:22,:); % template's CoR
%B = []; %import data from excel sheet

figure()
view([-4,1]);
hold on
axis equal
axis off
%plot3(Pjt_pts(:,1),Pjt_pts(:,2),Pjt_pts(:,3),'ko')
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

for i = 1:30
centers(i,:) = mesh.spheres{1,i}.center;
end
A = centers(1:22,:); % update A value after scale adjustment
visualization; update_cor_plot;
fprintf('scale adjustment is completed\n');

%% MCP Abduction/Adduction detection & transform - Part1

load('assignment_new.mat');
axes_d = bone_axes(mesh.spheres);

% set angle range = according to the posture #
o = A(16,:);
pt2 = A(16,:) + axes_d{6}(1 : 3, 1)';
pt3 = A(16,:) + axes_d{6}(1 : 3, 2)';
target_pt = B(15,:);
Pjt_pt1 = vector_projection(o, pt2, pt3, target_pt);
v1 = A(16,:)-A(15,:); v1 = v1/norm(v1);
v2 = A(16,:)-Pjt_pt1; v2 = v2/norm(v2);
cosTH = dot(v1,v2)/(norm(v1)*norm(v2));
TH1_abad = acos(cosTH);

o = A(12,:);
pt2 = A(12,:) + axes_d{9}(1 : 3, 1)';
pt3 = A(12,:) + axes_d{9}(1 : 3, 2)';
target_pt = B(11,:);
Pjt_pt2 = vector_projection(o, pt2, pt3, target_pt);
v1 = A(12,:)-A(11,:); v1 = v1/norm(v1);
v2 = A(12,:)-Pjt_pt2; v2 = v2/norm(v2);
cosTH = dot(v1,v2)/(norm(v1)*norm(v2));
TH2_abad = acos(cosTH);

o = A(8,:);
pt2 = A(8,:) + axes_d{12}(1 : 3, 1)';
pt3 = A(8,:) + axes_d{12}(1 : 3, 2)';
target_pt = B(7,:);
Pjt_pt3 = vector_projection(o, pt2, pt3, target_pt);
v1 = A(8,:)-A(7,:); v1 = v1/norm(v1);
v2 = A(8,:)-Pjt_pt3; v2 = v2/norm(v2);
cosTH = dot(v1,v2)/(norm(v1)*norm(v2));
TH3_abad = acos(cosTH);

o = A(4,:);
pt2 = A(4,:) + axes_d{12}(1 : 3, 1)';
pt3 = A(4,:) + axes_d{12}(1 : 3, 2)';
target_pt = B(3,:);
Pjt_pt4 = vector_projection(o, pt2, pt3, target_pt);
v1 = A(4,:)-A(3,:); v1 = v1/norm(v1);
v2 = A(4,:)-Pjt_pt4; v2 = v2/norm(v2);
cosTH = dot(v1,v2)/(norm(v1)*norm(v2));
TH4_abad = acos(cosTH);

Pjt_pts = [Pjt_pt1; Pjt_pt2; Pjt_pt3; Pjt_pt4];

figure()
view([-2,0]);
hold on
axis equal
axis off
plot3(Pjt_pts(:,1),Pjt_pts(:,2),Pjt_pts(:,3),'ko')
plot_AB;
hold off

%% MCP Abduction/Adduction detection & transform - Part2

% adjust rotation factor
% dorsal plane based rotation input
rotating_factor1 = +1;
rotating_factor2 = +1;
rotating_factor3 = +1;
rotating_factor4 = +1;

angle = zeros(19,1);
angle_record = zeros(19,1);
angle(16) = TH1_abad * rotating_factor1;
angle(17) = TH2_abad * rotating_factor2;
angle(18) = TH3_abad * rotating_factor3;
angle(19) = TH4_abad * rotating_factor4;
angle_record(16:19,1) = [angle(16); angle(17); angle(18); angle(19)];
mesh = transform_angle(mesh, angle);
visualization;
update_cor_plot;

fprintf('MCP abduction/adduction is completed\n');

%% MCP flexion/extension detection

v1 = A(16,:)-A(15,:); v1 = v1/norm(v1);
v2 = A(16,:)-B(15,:); v2 = v2/norm(v2);
cosTH = dot(v1,v2)/(norm(v1)*norm(v2));
TH1 = acos(cosTH);

v1 = A(12,:)-A(11,:); v1 = v1/norm(v1);
v2 = A(12,:)-B(11,:); v2 = v2/norm(v2);
cosTH = dot(v1,v2)/(norm(v1)*norm(v2));
TH2 = acos(cosTH);

v1 = A(8,:)-A(7,:); v1 = v1/norm(v1);
v2 = A(8,:)-B(7,:); v2 = v2/norm(v2);
cosTH = dot(v1,v2)/(norm(v1)*norm(v2));
TH3 = acos(cosTH);

v1 = A(4,:)-A(3,:); v1 = v1/norm(v1);
v2 = A(4,:)-B(3,:); v2 = v2/norm(v2);
cosTH = dot(v1,v2)/(norm(v1)*norm(v2));
TH4 = acos(cosTH);

rotating_factor1 = +1;
rotating_factor2 = +1;
rotating_factor3 = +1;
rotating_factor4 = +1;

angle = zeros(19,1);
angle(4) = TH1 * rotating_factor1;
angle(7) = TH2 * rotating_factor2;
angle(10) = TH3 * rotating_factor3;
angle(13) = TH4 * rotating_factor4;
angle_record([4 7 10 13],1) = [angle(4); angle(7); angle(10); angle(13)];
mesh = transform_angle(mesh, angle);
visualization;
update_cor_plot;

% v1 = A(4,:)-A(3,:); v1 = v1/norm(v1);
% v2 = A(4,:)-B(3,:); v2 = v2/norm(v2);
% cosTH = dot(v1,v2)/(norm(v1)*norm(v2));
% TH4 = acos(cosTH);
% angle = zeros(19,1);
% angle(13) = TH4 * rotating_factor4;
% angle_record(13,1) = angle_record(13,1) + angle(13);
% mesh = transform_angle(mesh, angle);
% visualization;
% update_cor_plot;

fprintf('MCP flexion/extension is completed\n');

%% PIP flexion/extension detection

v1 = A(15,:)-A(14,:); v1 = v1/norm(v1);
v2 = A(15,:)-B(14,:); v2 = v2/norm(v2);
cosTH = dot(v1,v2)/(norm(v1)*norm(v2));
TH5 = acos(cosTH);

v1 = A(11,:)-A(10,:); v1 = v1/norm(v1);
v2 = A(11,:)-B(10,:); v2 = v2/norm(v2);
cosTH = dot(v1,v2)/(norm(v1)*norm(v2));
TH6 = acos(cosTH);

v1 = A(7,:)-A(6,:); v1 = v1/norm(v1);
v2 = A(7,:)-B(6,:); v2 = v2/norm(v2);
cosTH = dot(v1,v2)/(norm(v1)*norm(v2));
TH7 = acos(cosTH);

v1 = A(3,:)-A(2,:); v1 = v1/norm(v1);
v2 = A(3,:)-B(2,:); v2 = v2/norm(v2);
cosTH = dot(v1,v2)/(norm(v1)*norm(v2));
TH8 = acos(cosTH);

rotating_factor1 = +1;
rotating_factor2 = +1;
rotating_factor3 = +1;
rotating_factor4 = +1;

angle = zeros(19,1);
angle(5) = TH5 * rotating_factor1;
angle(8) = TH6 * rotating_factor2;
angle(11) = TH7 * rotating_factor3;
angle(14) = TH8 * rotating_factor4;
angle_record([5 8 11 14],1) = [angle(5); angle(8); angle(11); angle(14)];
mesh = transform_angle(mesh, angle);
visualization;
update_cor_plot;

fprintf('PIP flexion/extension is completed\n');

%% PIP based MCP supination/pronation detection & transform

v1 = A(15,:)-A(14,:); v1 = v1/norm(v1);
v2 = A(15,:)-B(14,:); v2 = v2/norm(v2);
cosTH = dot(v1,v2)/(norm(v1)*norm(v2));
TH1_supr = acos(cosTH);

v1 = A(11,:)-A(10,:); v1 = v1/norm(v1);
v2 = A(11,:)-B(10,:); v2 = v2/norm(v2);
cosTH = dot(v1,v2)/(norm(v1)*norm(v2));
TH2_supr = acos(cosTH);

v1 = A(7,:)-A(6,:); v1 = v1/norm(v1);
v2 = A(7,:)-B(6,:); v2 = v2/norm(v2);
cosTH = dot(v1,v2)/(norm(v1)*norm(v2));
TH3_supr = acos(cosTH);

v1 = A(3,:)-A(2,:); v1 = v1/norm(v1);
v2 = A(3,:)-B(2,:); v2 = v2/norm(v2);
cosTH = dot(v1,v2)/(norm(v1)*norm(v2));
TH4_supr = acos(cosTH);

% input rotation factor at palmar side hand

rotating_factor1 = +1;
rotating_factor2 = +1;
rotating_factor3 = +1;
rotating_factor4 = +1;

angle_supr = zeros(4,1);
angle_supr_record = zeros(4,1);
angle_supr(1) = TH1_supr * rotating_factor1;
angle_supr(2) = TH2_supr * rotating_factor2;
angle_supr(3) = TH3_supr * rotating_factor3;
angle_supr(4) = TH4_supr * rotating_factor4;
angle_supr_record = angle_supr;
mesh = transform_sup_pro_angle(mesh, -angle_supr);
update_cor_plot;
visualization;

fprintf('PIP based MCP supination/pronation is completed\n');

%% DIP flexion/extension detection & transform

v1 = A(14,:)-A(13,:); v1 = v1/norm(v1);
v2 = A(14,:)-B(13,:); v2 = v2/norm(v2);
cosTH = dot(v1,v2)/(norm(v1)*norm(v2));
TH9 = acos(cosTH);

v1 = A(10,:)-A(9,:); v1 = v1/norm(v1);
v2 = A(10,:)-B(9,:); v2 = v2/norm(v2);
cosTH = dot(v1,v2)/(norm(v1)*norm(v2));
TH10 = acos(cosTH);

v1 = A(6,:)-A(5,:); v1 = v1/norm(v1);
v2 = A(6,:)-B(5,:); v2 = v2/norm(v2);
cosTH = dot(v1,v2)/(norm(v1)*norm(v2));
TH11 = acos(cosTH);

v1 = A(2,:)-A(1,:); v1 = v1/norm(v1);
v2 = A(2,:)-B(1,:); v2 = v2/norm(v2);
cosTH = dot(v1,v2)/(norm(v1)*norm(v2));
TH12 = acos(cosTH);

rotating_factor1 = +1;
rotating_factor2 = +1;
rotating_factor3 = +1;
rotating_factor4 = +1;

angle = zeros(19,1);
angle(6) = TH9 * rotating_factor1;
angle(9) = TH10 * rotating_factor2;
angle(12) = TH11 * rotating_factor3;
angle(15) = TH12 * rotating_factor4;
angle_record([6 9 12 15],1) = [angle(6); angle(9); angle(12); angle(15)];
mesh = transform_angle(mesh, angle);
update_cor_plot;
visualization;

fprintf('DIP flexion/extension is completed\n');

%% Things to do (thumb registration - temporal) 

load('assignment_new.mat');

%tic

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

fprintf('Thumb joint fitting is completed\n');

%toc

%% Non-rigid Registration (ICP)

targetV = points.vertices;
sourceV = mesh.vertices;
targetF = points.faces;
sourceF = mesh.faces;
iterations = 30;
flag_prealligndata = 1;
figureOn = 1;
rigidICP = 0;

[sourceV] = ICP_nonrigidICP(targetV, sourceV, targetF, sourceF, iterations, flag_prealligndata, figureOn, rigidICP);

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

fprintf('Non-rigid ICP registration is completed\n');

%% Save vertices

ZC_pos7_vertices = sourceV; 
mesh.vertices = ZC_pos7_vertices;
visualization
save ZC_pos7_vertices.mat ZC_pos7_vertices;
save ZC_pos7_mesh.mat mesh;

fprintf('Saving vertices and mesh is completed\n');

% p = 'JW';
% formatSpec = '%s_pos%d_%s';
% A2 = 'vertices';
% filename = sprintf(formatSpec,p,iii,A2);

%% things to do - need fine fitting??

% ============================== updated up to here  =====================
% update template's rotation axis based on the scan's axis
% apply fine fitting? with small range of motion
% ============================== jump to ICP =============================

%%

MCP_initial = 0.0;
ag_rg1 = 0.5; % MCP fx/ex
intv1 = 0.0167; % 1 deg.

% D2 MCP fit
angle = zeros(19,1);
FRP_dorsal_segment = 109;
record = [];
for ag2 = -ag_rg1:intv1:ag_rg1 % size = 60
    angle = zeros(19,1);
    angle(4) = MCP_initial + ag2;
    tr_mesh = transform_angle(mesh, angle);
    mean_distance = mean_dist_tester(tr_mesh, points, FRP_dorsal_segment, assignment_new);
    record = [record; MCP_initial+ag2 mean_distance];
end
[~,col] = min(record(:,2));
angle = zeros(19,1);
angle(4) = record(col,1);
mesh = transform_angle(mesh, angle); % adjust MCP posture & update mesh
angle_record(4) = angle_record(4) + angle(4);

% D3 MCP fit
FRP_dorsal_segment = 112;
record = [];
for ag2 = -ag_rg1:intv1:ag_rg1 % size = 48
    angle = zeros(19,1);
    angle(7) = MCP_initial + ag2;
    tr_mesh = transform_angle(mesh, angle);
    mean_distance = mean_dist_tester(tr_mesh, points, FRP_dorsal_segment, assignment_new);
    record = [record; MCP_initial+ag2 mean_distance];
end
[~,col] = min(record(:,2));
angle = zeros(19,1);
angle(7) = record(col,1);
mesh = transform_angle(mesh, angle); % adjust MCP posture & update mesh
angle_record(7) = angle_record(7) + angle(7);

% D4 MCP fit
FRP_dorsal_segment = 115;
record = [];
for ag2 = -ag_rg1:intv1:ag_rg1  % size = 48
    angle = zeros(19,1);
    angle(10) = MCP_initial+ag2;
    tr_mesh = transform_angle(mesh, angle);
    mean_distance = mean_dist_tester(tr_mesh, points, FRP_dorsal_segment, assignment_new);
    record = [record; MCP_initial+ag2 mean_distance];
end
[~,col] = min(record(:,2));
angle = zeros(19,1);
angle(10) = record(col,1);
mesh = transform_angle(mesh, angle); % adjust MCP posture & update mesh
angle_record(10) = angle_record(10) + angle(10);

rotating_factor1 = +1;
rotating_factor2 = -1;
rotating_factor3 = -1;

v1 = A(16,:)-A(15,:); v1 = v1/norm(v1);
v2 = A(16,:)-B(15,:); v2 = v2/norm(v2);
cosTH = dot(v1,v2)/(norm(v1)*norm(v2));
TH4 = acos(cosTH);

v1 = A(12,:)-A(11,:); v1 = v1/norm(v1);
v2 = A(12,:)-B(11,:); v2 = v2/norm(v2);
cosTH = dot(v1,v2)/(norm(v1)*norm(v2));
TH5 = acos(cosTH);

v1 = A(8,:)-A(7,:); v1 = v1/norm(v1);
v2 = A(8,:)-B(7,:); v2 = v2/norm(v2);
cosTH = dot(v1,v2)/(norm(v1)*norm(v2));
TH6 = acos(cosTH);

angle = zeros(19,1);
angle(16:18,1) = [rotating_factor1*TH4; rotating_factor2*TH5; rotating_factor3*TH6;]; % MCP abduction/adduction angle record
mesh = transform_angle(mesh, angle);

%% 
% D5 MCP fit
FRP_dorsal_segment = 118;
record = [];
for ag2 = -ag_rg1*2:intv1:ag_rg1*2 % size = 48
    angle = zeros(19,1);
    angle(13) = MCP_initial+ag2;
    tr_mesh = transform_angle(mesh, angle);
    mean_distance = mean_dist_tester(tr_mesh, points, FRP_dorsal_segment, assignment_new);
    record = [record; MCP_initial+ag2 mean_distance];
end
[~,col] = min(record(:,2));
angle = zeros(19,1);
angle(13) = record(col,1);
mesh = transform_angle(mesh, angle); % adjust MCP posture & update mesh
angle_record(13) = angle_record(13) + angle(13);

%% MCP adjustment code - valid when MCP flexion/extension if aligned

% rotation direction of MCP
% D2 MCP = +: ccw , -: cw
% D3 MCP = +: ccw , -: cw
% D4 MCP = +: ccw , -: cw
% D5 MCP = +: ccw , -: cw

% A update
centers = zeros(30,3);
for i = 1:30
centers(i,:) = mesh.spheres{1,i}.center;
end
A = centers(1:22,:); % template's CoR

figure()
axis equal
axis off
hold on
plot_AB
hold off

rotating_factor1 = +1;
rotating_factor2 = -1;
rotating_factor3 = -1;
rotating_factor4 = -1;

v1 = A(16,:)-A(15,:); v1 = v1/norm(v1);
v2 = A(16,:)-B(15,:); v2 = v2/norm(v2);
cosTH = dot(v1,v2)/(norm(v1)*norm(v2));
TH1 = acos(cosTH);

v1 = A(12,:)-A(11,:); v1 = v1/norm(v1);
v2 = A(12,:)-B(11,:); v2 = v2/norm(v2);
cosTH = dot(v1,v2)/(norm(v1)*norm(v2));
TH2 = acos(cosTH);

v1 = A(8,:)-A(7,:); v1 = v1/norm(v1);
v2 = A(8,:)-B(7,:); v2 = v2/norm(v2);
cosTH = dot(v1,v2)/(norm(v1)*norm(v2));
TH3 = acos(cosTH);

v1 = A(4,:)-A(3,:); v1 = v1/norm(v1);
v2 = A(4,:)-B(3,:); v2 = v2/norm(v2);
cosTH = dot(v1,v2)/(norm(v1)*norm(v2));
TH4 = acos(cosTH);

angle = zeros(19,1);
angle(16:19,1) = [rotating_factor1*TH1; rotating_factor2*TH2; rotating_factor3*TH3; rotating_factor4*TH4]; % MCP abduction/adduction angle record
mesh = transform_angle(mesh, angle);

%% PIP fit
load('assignment_new.mat');
update_cor_plot;
visualization;

v1 = A(15,:)-A(14,:); v1 = v1/norm(v1);
v2 = A(15,:)-B(14,:); v2 = v2/norm(v2);
cosTH = dot(v1,v2)/(norm(v1)*norm(v2));
TH_PIP_1 = acos(cosTH);

v1 = A(11,:)-A(10,:); v1 = v1/norm(v1);
v2 = A(11,:)-B(10,:); v2 = v2/norm(v2);
cosTH = dot(v1,v2)/(norm(v1)*norm(v2));
TH_PIP_2 = acos(cosTH);

v1 = A(7,:)-A(6,:); v1 = v1/norm(v1);
v2 = A(7,:)-B(6,:); v2 = v2/norm(v2);
cosTH = dot(v1,v2)/(norm(v1)*norm(v2));
TH_PIP_3 = acos(cosTH);

v1 = A(3,:)-A(2,:); v1 = v1/norm(v1);
v2 = A(3,:)-B(2,:); v2 = v2/norm(v2);
cosTH = dot(v1,v2)/(norm(v1)*norm(v2));
TH_PIP_4 = acos(cosTH);

angle = zeros(19,1);
angle(5) = TH_PIP_1;
angle(8) = TH_PIP_2;
angle(11) = TH_PIP_3;
angle(14) = TH_PIP_4;
angle_record([5 8 11 14],1) = [TH_PIP_1; TH_PIP_2; TH_PIP_3; TH_PIP_4];
mesh = transform_angle(mesh, angle);
update_cor_plot;
visualization;

rotating_factor5 = -1;
rotating_factor6 = -1;
rotating_factor7 = -1;
rotating_factor8 = +1;

v1 = A(15,:)-A(14,:); v1 = v1/norm(v1);
v2 = A(15,:)-B(14,:); v2 = v2/norm(v2);
cosTH = dot(v1,v2)/(norm(v1)*norm(v2));
TH_PIP_5 = acos(cosTH);

v1 = A(11,:)-A(10,:); v1 = v1/norm(v1);
v2 = A(11,:)-B(10,:); v2 = v2/norm(v2);
cosTH = dot(v1,v2)/(norm(v1)*norm(v2));
TH_PIP_6 = acos(cosTH);

v1 = A(7,:)-A(6,:); v1 = v1/norm(v1);
v2 = A(7,:)-B(6,:); v2 = v2/norm(v2);
cosTH = dot(v1,v2)/(norm(v1)*norm(v2));
TH_PIP_7 = acos(cosTH);

v1 = A(3,:)-A(2,:); v1 = v1/norm(v1);
v2 = A(3,:)-B(2,:); v2 = v2/norm(v2);
cosTH = dot(v1,v2)/(norm(v1)*norm(v2));
TH_PIP_8 = acos(cosTH);

angle_supr = zeros(4,1);
angle_supr_record = zeros(4,1);
angle_supr(1) = TH_PIP_5 * rotating_factor5;
angle_supr(2) = TH_PIP_6 * rotating_factor6;
angle_supr(3) = TH_PIP_7 * rotating_factor7;
angle_supr(4) = TH_PIP_8 * rotating_factor8;
angle_supr_record(1:4,1) = [TH_PIP_5; TH_PIP_6; TH_PIP_7; TH_PIP_8];
mesh = transform_sup_pro_angle(mesh, -angle_supr);
update_cor_plot;
visualization;

%% ============================== start here

% set angle range = according to the posture #

% MCP flexion/extension guide
% posture 1 : -30 deg ~ 30 deg ==> -0.5 ~ 0.5
temp2 = zeros(1,4); % PIP flexion/extension angle record
PIP_initial2 = 1.5;
PIP_initial3 = 1.5;
PIP_initial4 = 1.5;
PIP_initial5 = 1.5;
ag_rg2 = 0.5;
intv2 = 0.0167;


% D2 PIP fit
FRP_dorsal_segment = 110;
record = [];
for ag3 = -ag_rg2:intv2:ag_rg2 % size = 60
        angle = zeros(19,1);
        angle(5) = PIP_initial2 + ag3;
        tr_mesh = transform_angle(mesh, angle);
        mean_distance = mean_dist_tester(tr_mesh, points, FRP_dorsal_segment, assignment_new);
        record = [record; PIP_initial2+ag3 mean_distance];
end
[~,col] = min(record(:,2));
angle = zeros(19,1);
angle(5) = record(col,1);
mesh = transform_angle(mesh, angle); % adjust MCP posture & update mesh
temp2(1) = angle(5);

% D3 PIP fit
FRP_dorsal_segment = 113;
record = [];
for ag3 = -ag_rg2:intv2:ag_rg2 % size = 67
        angle = zeros(19,1);
        angle(8) = PIP_initial3 + ag3;
        tr_mesh = transform_angle(mesh, angle);
        mean_distance = mean_dist_tester(tr_mesh, points, FRP_dorsal_segment, assignment_new);
        record = [record; PIP_initial3+ag3 mean_distance];
end
[~,col] = min(record(:,2));
angle = zeros(19,1);
angle(8) = record(col,1);
mesh = transform_angle(mesh, angle); % adjust MCP posture & update mesh
temp2(2) = angle(8);

% D4 PIP fit
FRP_dorsal_segment = 116;
record = [];
for ag3 = -ag_rg2:intv2:ag_rg2 % size = 67
        angle = zeros(19,1);
        angle(11) = PIP_initial4 + ag3;
        tr_mesh = transform_angle(mesh, angle);
        mean_distance = mean_dist_tester(tr_mesh, points, FRP_dorsal_segment, assignment_new);
        record = [record; PIP_initial4+ag3 mean_distance];
end
[~,col] = min(record(:,2));
angle = zeros(19,1);
angle(11) = record(col,1);
mesh = transform_angle(mesh, angle); % adjust MCP posture & update mesh
temp2(3) = angle(11);

% D5 PIP fit
FRP_dorsal_segment = 119;
record = [];
for ag3 = -ag_rg2:intv2:ag_rg2 % size = 67
        angle = zeros(19,1);
        angle(14) = PIP_initial5 + ag3;
        tr_mesh = transform_angle(mesh, angle);
        mean_distance = mean_dist_tester(tr_mesh, points, FRP_dorsal_segment, assignment_new);
        record = [record; PIP_initial5+ag3 mean_distance];
end
[~,col] = min(record(:,2));
angle = zeros(19,1);
angle(14) = record(col,1);
mesh = transform_angle(mesh, angle); % adjust MCP posture & update mesh
temp2(4) = angle(14);

toc

%% PIP base supination fit

tic
% D3 Register

% M1 = [-ag_rg1:intv1:ag_rg1];
% M2 = [-ag_rg2:intv2:ag_rg2];
% M3 = [-0.15:intv1:0.15];
% M4 = [-(ag_rg3/3):intv3:ag_rg3];
% 
% calc_size = size(M1,2) + size(M2,2) + size(M3,2) + size(M4,2);
% fprintf('calculation size is %.0f\n',calc_size);

% D2 MCP supination/pronation
temp3 = zeros(1,4); % PIP based MCP supination/pronation angle record
FRP_dorsal_segment = 110;
record = [];
for ag3 = -0.4:intv1:0.4 % size = 24
        angle_supr = zeros(4,1);
        angle_supr(1) = ag3;
        tr_mesh = transform_sup_pro_angle(mesh, angle_supr);
        mean_distance = mean_dist_tester(tr_mesh, points, FRP_dorsal_segment, assignment_new);
        record = [record; ag3 mean_distance];
end
[~,col] = min(record(:,2));
angle_supr = zeros(4,1);
angle_supr(1) = record(col,1);
mesh = transform_sup_pro_angle(mesh, angle_supr); % adjust MCP posture & update mesh
temp3(1) = angle_supr(1);

% D3 MCP supination/pronation
FRP_dorsal_segment = 113;
record = [];
for ag3 = -0.3:intv1:0.3 % size = 9
        angle_supr = zeros(4,1);
        angle_supr(2) = ag3;
        tr_mesh = transform_sup_pro_angle(mesh, angle_supr);
        mean_distance = mean_dist_tester(tr_mesh, points, FRP_dorsal_segment, assignment_new);
        record = [record; ag3 mean_distance];
end
[~,col] = min(record(:,2));
angle_supr = zeros(4,1);
angle_supr(2) = record(col,1);
mesh = transform_sup_pro_angle(mesh, angle_supr); % adjust MCP posture & update mesh
temp3(2) = angle_supr(2);

% D4 MCP supination/pronation
FRP_dorsal_segment = 116;
record = [];
for ag3 = -0.28:intv1:0.28 % size = 9
        angle_supr = zeros(4,1);
        angle_supr(3) = ag3;
        tr_mesh = transform_sup_pro_angle(mesh, angle_supr);
        mean_distance = mean_dist_tester(tr_mesh, points, FRP_dorsal_segment, assignment_new);
        record = [record; ag3 mean_distance];
end
[~,col] = min(record(:,2));
angle_supr = zeros(4,1);
angle_supr(3) = record(col,1);
mesh = transform_sup_pro_angle(mesh, angle_supr); % adjust MCP posture & update mesh
temp3(3) = angle_supr(3);

% D5 MCP supination/pronation
FRP_dorsal_segment = 119;
record = [];
for ag3 = -0.3:intv1:0.3 % size = 9
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
temp3(4) = angle_supr(4);

toc

%% DIP fit - initial angle = ?
tic

DIP_initial = 1.5;
ag_rg3 = 0.5;
intv3 = 0.0167;

% D2 DIP
FRP_dorsal_segment = 111;
record = [];
for ag3 = -(ag_rg3/3):intv3:ag_rg3 % size = 40
        angle = zeros(19,1);
        angle(6) = DIP_initial + ag3;
        tr_mesh = transform_angle(mesh, angle);
        mean_distance = mean_dist_tester(tr_mesh, points, FRP_dorsal_segment, assignment_new);
        record = [record; DIP_initial + ag3 mean_distance];
end
[~,col] = min(record(:,2));
angle = zeros(19,1);
angle(6) = record(col,1);
mesh = transform_angle(mesh, angle); % adjust MCP posture & update mesh

% D3 DIP fit
FRP_dorsal_segment = 114;
record = [];
for ag3 = -(ag_rg3/3):intv3:ag_rg3 % size = 26
        angle = zeros(19,1);
        angle(9) = DIP_initial + ag3;
        tr_mesh = transform_angle(mesh, angle);
        mean_distance = mean_dist_tester(tr_mesh, points, FRP_dorsal_segment, assignment_new);
        record = [record; DIP_initial + ag3 mean_distance];
end
[~,col] = min(record(:,2));
angle = zeros(19,1);
angle(9) = record(col,1);
mesh = transform_angle(mesh, angle); % adjust MCP posture & update mesh

% D4 DIP fit
FRP_dorsal_segment = 117;
record = [];
for ag3 = -0.4:0.0167:0.4 % size = 26
        angle = zeros(19,1);
        angle(12) = DIP_initial + ag3;
        tr_mesh = transform_angle(mesh, angle);
        mean_distance = mean_dist_tester(tr_mesh, points, FRP_dorsal_segment, assignment_new);
        record = [record; DIP_initial + ag3 mean_distance];
end
[~,col] = min(record(:,2));
angle = zeros(19,1);
angle(12) = record(col,1);
mesh = transform_angle(mesh, angle); % adjust MCP posture & update mesh
%

% D5 DIP
FRP_dorsal_segment = 120;
record = [];
for ag3 = -0.4:0.0167:0.4 % size = 26
        angle = zeros(19,1);
        angle(15) = DIP_initial + ag3;
        tr_mesh = transform_angle(mesh, angle);
        mean_distance = mean_dist_tester(tr_mesh, points, FRP_dorsal_segment, assignment_new);
        record = [record; DIP_initial + ag3 mean_distance];
end
[~,col] = min(record(:,2));
angle = zeros(19,1);
angle(15) = record(col,1);
mesh = transform_angle(mesh, angle); % adjust MCP posture & update mesh

toc

%% DIP base supination fit

tic

% D2 MCP supination/pronation
FRP_dorsal_segment = 111;
record = [];
for ag3 = -0.28:intv1:0.28 % size = 24
        angle_supr = zeros(4,1);
        angle_supr(1) = ag3;
        tr_mesh = transform_sup_pro_angle(mesh, angle_supr);
        mean_distance = mean_dist_tester(tr_mesh, points, FRP_dorsal_segment, assignment_new);
        record = [record; ag3 mean_distance];
end
[~,col] = min(record(:,2));
angle_supr = zeros(4,1);
angle_supr(1) = record(col,1);
mesh = transform_sup_pro_angle(mesh, angle_supr); % adjust MCP posture & update mesh

% D3 MCP supination/pronation
FRP_dorsal_segment = 114;
record = [];
for ag3 = -0.15:intv1:0.15 % size = 9
        angle_supr = zeros(4,1);
        angle_supr(2) = ag3;
        tr_mesh = transform_sup_pro_angle(mesh, angle_supr);
        mean_distance = mean_dist_tester(tr_mesh, points, FRP_dorsal_segment, assignment_new);
        record = [record; ag3 mean_distance];
end
[~,col] = min(record(:,2));
angle_supr = zeros(4,1);
angle_supr(2) = record(col,1);
mesh = transform_sup_pro_angle(mesh, angle_supr); % adjust MCP posture & update mesh

% D4 MCP supination/pronation
FRP_dorsal_segment = 117;
record = [];
for ag3 = -0.28:0.0167:0.28 % size = 9
        angle_supr = zeros(4,1);
        angle_supr(3) = ag3;
        tr_mesh = transform_sup_pro_angle(mesh, angle_supr);
        mean_distance = mean_dist_tester(tr_mesh, points, FRP_dorsal_segment, assignment_new);
        record = [record; ag3 mean_distance];
end
[~,col] = min(record(:,2));
angle_supr = zeros(4,1);
angle_supr(3) = record(col,1);
mesh = transform_sup_pro_angle(mesh, angle_supr); % adjust MCP posture & update mesh

% D5 MCP supination/pronation
FRP_dorsal_segment = 120;
record = [];
for ag3 = -0.28:0.0167:0.28 % size = 9
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

toc


































