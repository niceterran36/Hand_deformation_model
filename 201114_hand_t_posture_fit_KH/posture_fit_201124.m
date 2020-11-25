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

%% Load data
global assignment_new
load('hy_mesh_n5.mat'); %template
load('assignment_new.mat');
[points.vertices, points.faces, points.FB, points.H] = function_loading_ply_file('JB_pos9.ply'); % target scan
points.normals = per_vertex_normals(points.vertices, points.faces);
LMs_PLM = function_get_LM_from_iges('JB_pos9_PLM.igs'); % LM for scan
LMt_PLM = function_get_LM_from_iges('LMt.igs'); % LM for template


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

%% MCP flexion/extension detection

load('assignment_new.mat');

% set angle range = according to the posture #

% MCP flexion/extension guide
% posture 1 : -30 deg ~ 30 deg ==> -0.5 ~ 0.5

MCP_initial = 0.8;
ag_rg1 = 0.5; % MCP fx/ex
intv1 = 0.0167; % 1 deg.

intv0 = 0.0167; % 1 deg.
lb1 = 0;
up1 = 1;


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

% D5 MCP fit
FRP_dorsal_segment = 118;
record = [];
for ag2 = -ag_rg1:intv1:ag_rg1 % size = 48
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

% D2 MCP fit
% angle = zeros(19,1);
% FRP_dorsal_segment = 109;
% record = [];
% for ag2 = -ag_rg1:intv1:ag_rg1 % size = 60
%     angle = zeros(19,1);
%     angle(4) = ag2;
%     tr_mesh = transform_angle(mesh, angle);
%     mean_distance = mean_dist_tester(tr_mesh, points, FRP_dorsal_segment, assignment_new);
%     record = [record; ag2 mean_distance];
% end
% [~,col] = min(record(:,2));
% angle = zeros(19,1);
% angle(4) = record(col,1);
% mesh = transform_angle(mesh, angle); % adjust MCP posture & update mesh

% D4 MCP fit
% FRP_dorsal_segment = 115;
% record = [];
% for ag2 = -0.4:0.0167:0.4 % size = 48
%     angle = zeros(19,1);
%     angle(10) = ag2;
%     tr_mesh = transform_angle(mesh, angle);
%     mean_distance = mean_dist_tester(tr_mesh, points, FRP_dorsal_segment, assignment_new);
%     record = [record; ag2 mean_distance];
% end
% [~,col] = min(record(:,2));
% angle = zeros(19,1);
% angle(10) = record(col,1);
% mesh = transform_angle(mesh, angle); % adjust MCP posture & update mesh



% D3 MCP fit
% FRP_dorsal_segment = 112;
% record = [];
% for ag2 = -ag_rg1:intv1:ag_rg1 % size = 48
%     angle = zeros(19,1);
%     angle(7) = ag2;
%     tr_mesh = transform_angle(mesh, angle);
%     mean_distance = mean_dist_tester(tr_mesh, points, FRP_dorsal_segment, assignment_new);
%     record = [record; ag2 mean_distance];
% end
% [~,col] = min(record(:,2));
% angle = zeros(19,1);
% angle(7) = record(col,1);
% mesh = transform_angle(mesh, angle); % adjust MCP posture & update mesh


% D5 MCP fit
% FRP_dorsal_segment = 118;
% record = [];
% for ag2 = -0.4:0.0167:0.4 % size = 48
%     angle = zeros(19,1);
%     angle(13) = ag2;
%     tr_mesh = transform_angle(mesh, angle);
%     mean_distance = mean_dist_tester(tr_mesh, points, FRP_dorsal_segment, assignment_new);
%     record = [record; ag2 mean_distance];
% end
% [~,col] = min(record(:,2));
% angle = zeros(19,1);
% angle(13) = record(col,1);
% mesh = transform_angle(mesh, angle); % adjust MCP posture & update mesh

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
angle(16:19,1) = [TH1; TH2; TH3; -TH4];
% angle(4) = -TH1
% angle(7) = -TH2
% angle(10) = -TH3
% angle(13) = -TH4/2
% angle(19) = -TH4
mesh = transform_angle(mesh, angle);



% % call MCP axes
% 
% org = axes{6}(1:3,4)';
% 
% A_tr_o = A - org;
% B_tr_o = B - org;
% 
% ax1 = axes{6}(1:3,1)';
% ax2 = axes{6}(1:3,2)';
% ax3 = axes{6}(1:3,3)'; % normal vector 
% 
% Bv = B(15,:)-A(16,:)
% Bp = B(15,:);
% %Bv = Bv/norm(Bv);
% Bv_p = Bv + dot(Bv,ax3)*(-1)*ax3
% Av = A(15,:)-A(16,:);
% %Av = Av/norm(Av);
% 
% ax1 = 20*ax1;
% ax2 = 20*ax2;
% ax3 = 20*ax3;
% ax1_tr = org + ax1;
% ax2_tr = org + ax2;
% ax3_tr = org + ax3;
% Bv_p_tr = org + Bv_p;
% AXS = [ax1_tr; ax2_tr; ax3_tr; org; Bv; Av; Bv_p_tr];
% 
% figure()
% axis equal
% axis off
% hold on
% plot_AB
% plot3(AXS([1 4],1),AXS([1 4],2),AXS([1 4],3),'-c');
% plot3(AXS([2 4],1),AXS([2 4],2),AXS([2 4],3),'-c');
% plot3(AXS([3 4],1),AXS([3 4],2),AXS([3 4],3),'-c');
% plot3(AXS(1:4,1),AXS(1:4,2),AXS(1:4,3),'c*');
% % plot3(AXS([4 5],1),AXS([4 5],2),AXS([4 5],3),'-r');
% % plot3(AXS([4 6],1),AXS([4 6],2),AXS([4 6],3),'-b');
% plot3(B(15,1),B(15,2),B(15,3),'bo');
% plot3(A(15,1),A(15,2),A(15,3),'ro');
% 
% % plot3(AXS(3,1),AXS(3,2),AXS(3,3),'mo');
% % plot3(AXS([3 4],1),AXS([3 4],2),AXS([3 4],3),'-m');
% % plot3(AXS(5,1),AXS(5,2),AXS(5,3),'ro');
% % plot3(AXS(6,1),AXS(6,2),AXS(6,3),'bo');
% 
% hold off
% 
% % point to plane distance = P-P0 dot plane normal 

save mesh_backup.mat mesh

%% PIP fit

tic 
load('assignment_new.mat');

% set angle range = according to the posture #

% MCP flexion/extension guide
% posture 1 : -30 deg ~ 30 deg ==> -0.5 ~ 0.5
ag_rg1 = 0.5; % MCP fx/ex
intv1 = 0.0167; % 1 deg.
ag_rg2 = 0.3;
intv2 = 0.0167;


PIP_initial = 1.6;
PIP_initial2 = 1.8;
PIP_initial3 = 2;
PIP_initial4 = 2;
PIP_initial5 = 1.8;


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
FRP_dorsal_segment = 110;
record = [];
for ag3 = -0.5:intv1*2:0.28 % size = 24
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
FRP_dorsal_segment = 113;
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
FRP_dorsal_segment = 116;
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
mesh = transform_sup_pro_angle(mesh, angle_supr); % adjust MCP posture & update mesh

% D5 MCP supination/pronation
FRP_dorsal_segment = 119;
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

toc

%% DIP fit - initial angle = ?
tic

DIP_initial = 0.8;
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
for ag3 = -0.5:intv1*2:0.28 % size = 24
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
mesh = transform_sup_pro_angle(mesh, angle_supr); % adjust MCP posture & update mesh

% D5 MCP supination/pronation
FRP_dorsal_segment = 120;
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

toc

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

%% Save vertices
JB_pos7_vertices = sourceV; 
mesh.vertices = JB_pos7_vertices;
visualization
save JB_pos7_vertices.mat JB_pos7_vertices;



































