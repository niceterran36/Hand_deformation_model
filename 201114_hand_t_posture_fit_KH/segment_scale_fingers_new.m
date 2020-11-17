function [mesh_tr] = segment_scale_fingers_new(mesh, LMt, LMs)
% jCOR1 = fixed COR, jCOR2 = transformed COR
% V = vertices
% F = faces
% CP1: template CoR groups
% CPs1: scan's CoR groups
% target_length = amount of how much stretch the mesh in terms of the link
% length 

D2_J_DIP_t = mean(LMt([1 13 16 43],:));
D2_J_PIP_t = mean(LMt([2 14 17 44],:));
D2_J_MCP_t = mean(LMt([3 15 18 45],:));

D3_J_DIP_t = mean(LMt([4 19 21 40],:));
D3_J_PIP_t = mean(LMt([5 20 22 41],:));
D3_J_MCP_t = mean(LMt([6 18 23 42],:));

D4_J_DIP_t = mean(LMt([7 24 26 37],:));
D4_J_PIP_t = mean(LMt([8 25 27 38],:));
D4_J_MCP_t = mean(LMt([9 23 28 39],:));

D5_J_DIP_t = mean(LMt([10 29 31 34],:));
D5_J_PIP_t = mean(LMt([11 30 32 35],:));
D5_J_MCP_t = mean(LMt([12 28 33 36],:));

D2_J_DIP_s = mean(LMs([1 13 16 43],:));
D2_J_PIP_s = mean(LMs([2 14 17 44],:));
D2_J_MCP_s = mean(LMs([3 15 18 45],:));

D3_J_DIP_s = mean(LMs([4 19 21 40],:));
D3_J_PIP_s = mean(LMs([5 20 22 41],:));
D3_J_MCP_s = mean(LMs([6 18 23 42],:));

D4_J_DIP_s = mean(LMs([7 24 26 37],:));
D4_J_PIP_s = mean(LMs([8 25 27 38],:));
D4_J_MCP_s = mean(LMs([9 23 28 39],:));

D5_J_DIP_s = mean(LMs([10 29 31 34],:));
D5_J_PIP_s = mean(LMs([11 30 32 35],:));
D5_J_MCP_s = mean(LMs([12 28 33 36],:));

% 
% srf_LMt = LMt(1:12,:); % palmar
% srf_LMs = LMs(1:12,:);
% 
% srf2_LMt = LMt(1:12,:); % dorsal

% Digit 2 re-scale
keep = ismember(mesh.assignment, 9:11);
[vertices, faces] = filter_vertices(mesh.vertices, mesh.faces, keep);

t_vector1 = D2_J_PIP_t - D2_J_MCP_t;
t_vector2 = D2_J_DIP_t - D2_J_PIP_t;
s_vector1 = D2_J_PIP_s - D2_J_MCP_s;
s_vector2 = D2_J_DIP_s - D2_J_PIP_s;

scale_factor1 = norm(s_vector1)/norm(t_vector1);
scale_factor2 = norm(s_vector2)/norm(t_vector2);

clear t_vector1 t_vector2 s_vector1 s_vector2

C1 = mesh.spheres{1,16}.center; 
C2 = mesh.spheres{1,15}.center; 
C3 = mesh.spheres{1,14}.center; 
C4 = mesh.spheres{1,13}.center; 
CP1 = [C1; C2; C3; C4];

vector1 = CP1(2,:) - CP1(1,:);
vector2 = CP1(3,:) - CP1(2,:);

vector1_n = scale_factor1*vector1;
vector2_n = scale_factor2*vector2;

C2_n = CP1(1,:) + vector1_n;
C3_n = C2_n + vector2_n;
C4_n = C3_n + vector2_n;
CP2 = [CP1(1,:); C2_n; C3_n; C4_n];

[b,bc] = bbw_boundary_conditions(vertices, faces, CP1(1:3,:));
W = bbw_biharmonic_bounded(vertices, faces, b, bc);
W = W./repmat(sum(W,2),1,size(W,2));
Diff = CP2(1:3,:) - CP1(1:3,:);
V_tr = bbw_simple_deform(vertices, faces, CP1(1:3,:), W, Diff);

mesh.vertices(keep,:) = V_tr;
mesh.spheres{1,15}.center = C2_n; 
mesh.spheres{1,14}.center = C3_n;
mesh.spheres{1,13}.center = C4_n;

clear b bc C1 C2 C2_n C3 C3_n C4 C4_n CP1 CP2 Diff V_tr vector1 vector1_n vector2 vector2_n W vertices faces keep

% Digit 3 re-scale
keep = ismember(mesh.assignment, 12:14);
[vertices, faces] = filter_vertices(mesh.vertices, mesh.faces, keep);

t_vector1 = D3_J_PIP_t - D3_J_MCP_t;
t_vector2 = D3_J_DIP_t - D3_J_PIP_t;
s_vector1 = D3_J_PIP_s - D3_J_MCP_s;
s_vector2 = D3_J_DIP_s - D3_J_PIP_s;

scale_factor1 = norm(s_vector1)/norm(t_vector1);
scale_factor2 = norm(s_vector2)/norm(t_vector2);

clear t_vector1 t_vector2 s_vector1 s_vector2

C1 = mesh.spheres{1,12}.center; 
C2 = mesh.spheres{1,11}.center; 
C3 = mesh.spheres{1,10}.center; 
C4 = mesh.spheres{1,9}.center; 
CP1 = [C1; C2; C3; C4];

vector1 = CP1(2,:) - CP1(1,:);
vector2 = CP1(3,:) - CP1(2,:);

vector1_n = scale_factor1*vector1;
vector2_n = scale_factor2*vector2;

C2_n = CP1(1,:) + vector1_n;
C3_n = C2_n + vector2_n;
C4_n = C3_n + vector2_n;
CP2 = [CP1(1,:); C2_n; C3_n; C4_n];

[b,bc] = bbw_boundary_conditions(vertices, faces, CP1(1:3,:));
W = bbw_biharmonic_bounded(vertices, faces, b, bc);
W = W./repmat(sum(W,2),1,size(W,2));
Diff = CP2(1:3,:) - CP1(1:3,:);
V_tr = bbw_simple_deform(vertices, faces, CP1(1:3,:), W, Diff);

mesh.vertices(keep,:) = V_tr;
mesh.spheres{1,11}.center = C2_n; 
mesh.spheres{1,10}.center = C3_n;
mesh.spheres{1,9}.center = C4_n;

clear b bc C1 C2 C2_n C3 C3_n C4 C4_n CP1 CP2 Diff V_tr vector1 vector1_n vector2 vector2_n W vertices faces keep

% Digit 4 re-scale
keep = ismember(mesh.assignment, 15:17);
[vertices, faces] = filter_vertices(mesh.vertices, mesh.faces, keep);

t_vector1 = D4_J_PIP_t - D4_J_MCP_t;
t_vector2 = D4_J_DIP_t - D4_J_PIP_t;
s_vector1 = D4_J_PIP_s - D4_J_MCP_s;
s_vector2 = D4_J_DIP_s - D4_J_PIP_s;

scale_factor1 = norm(s_vector1)/norm(t_vector1);
scale_factor2 = norm(s_vector2)/norm(t_vector2);

clear t_vector1 t_vector2 s_vector1 s_vector2

C1 = mesh.spheres{1,8}.center; 
C2 = mesh.spheres{1,7}.center; 
C3 = mesh.spheres{1,6}.center; 
C4 = mesh.spheres{1,5}.center; 
CP1 = [C1; C2; C3; C4];

vector1 = CP1(2,:) - CP1(1,:);
vector2 = CP1(3,:) - CP1(2,:);

vector1_n = scale_factor1*vector1;
vector2_n = scale_factor2*vector2;

C2_n = CP1(1,:) + vector1_n;
C3_n = C2_n + vector2_n;
C4_n = C3_n + vector2_n;
CP2 = [CP1(1,:); C2_n; C3_n; C4_n];

[b,bc] = bbw_boundary_conditions(vertices, faces, CP1(1:3,:));
W = bbw_biharmonic_bounded(vertices, faces, b, bc);
W = W./repmat(sum(W,2),1,size(W,2));
Diff = CP2(1:3,:) - CP1(1:3,:);
V_tr = bbw_simple_deform(vertices, faces, CP1(1:3,:), W, Diff);

mesh.vertices(keep,:) = V_tr;
mesh.spheres{1,7}.center = C2_n; 
mesh.spheres{1,6}.center = C3_n;
mesh.spheres{1,5}.center = C4_n;

clear b bc C1 C2 C2_n C3 C3_n C4 C4_n CP1 CP2 Diff V_tr vector1 vector1_n vector2 vector2_n W vertices faces keep

% Digit 5 re-scale
keep = ismember(mesh.assignment, 18:20);
[vertices, faces] = filter_vertices(mesh.vertices, mesh.faces, keep);

t_vector1 = D5_J_PIP_t - D5_J_MCP_t;
t_vector2 = D5_J_DIP_t - D5_J_PIP_t;
s_vector1 = D5_J_PIP_s - D5_J_MCP_s;
s_vector2 = D5_J_DIP_s - D5_J_PIP_s;

scale_factor1 = norm(s_vector1)/norm(t_vector1);
scale_factor2 = norm(s_vector2)/norm(t_vector2);

clear t_vector1 t_vector2 s_vector1 s_vector2

C1 = mesh.spheres{1,4}.center; 
C2 = mesh.spheres{1,3}.center; 
C3 = mesh.spheres{1,2}.center; 
C4 = mesh.spheres{1,1}.center; 
CP1 = [C1; C2; C3; C4];

vector1 = CP1(2,:) - CP1(1,:);
vector2 = CP1(3,:) - CP1(2,:);

vector1_n = scale_factor1*vector1;
vector2_n = scale_factor2*vector2;

C2_n = CP1(1,:) + vector1_n;
C3_n = C2_n + vector2_n;
C4_n = C3_n + vector2_n;
CP2 = [CP1(1,:); C2_n; C3_n; C4_n];

[b,bc] = bbw_boundary_conditions(vertices, faces, CP1(1:3,:));
W = bbw_biharmonic_bounded(vertices, faces, b, bc);
W = W./repmat(sum(W,2),1,size(W,2));
Diff = CP2(1:3,:) - CP1(1:3,:);
V_tr = bbw_simple_deform(vertices, faces, CP1(1:3,:), W, Diff);

mesh.vertices(keep,:) = V_tr;
mesh.spheres{1,3}.center = C2_n; 
mesh.spheres{1,2}.center = C3_n;
mesh.spheres{1,1}.center = C4_n;

clear b bc C1 C2 C2_n C3 C3_n C4 C4_n CP1 CP2 Diff V_tr vector1 vector1_n vector2 vector2_n W vertices faces keep

mesh_tr = mesh;

% visualize
% figure()
% hold on;
% view_angle = [207,10];
% view(view_angle);
% trimesh(mesh.faces, mesh.vertices(:, 1), mesh.vertices(:, 2), mesh.vertices(:, 3), 'EdgeColor', 'none', 'FaceColor', [0.4, 0.9, 0.4], 'FaceAlpha', 0.5);
% trimesh(points.faces, points.vertices(:, 1), points.vertices(:, 2), points.vertices(:, 3), 'EdgeColor', 'none', 'FaceColor', [0.8, 0.8, 0.8], 'FaceAlpha', 0.5);
% %quiver3(vertices(:, 1), vertices(:, 2), vertices(:, 3), normals(:, 1), normals(:, 2), normals(:, 3), 'Color', [0.4, 0.9, 0.4]);
% %quiver3(points.vertices(:, 1), points.vertices(:, 2), points.vertices(:, 3), points.normals(:, 1), points.normals(:, 2), points.normals(:, 3), 'Color', [0.8, 0.8, 0.8]);
% hold off;
% view([-90,0]);
% camlight;
% view([90, 0]);
% camlight;
% view(view_angle);
% axis equal;
% grid off;
% lighting gouraud;
% axis off;
% title('initial registration');
% 
% transformed = mesh_tr;
% 
% C1 = mesh.spheres{1,16}.center; 
% C2 = mesh.spheres{1,15}.center; 
% C3 = mesh.spheres{1,14}.center; 
% C4 = mesh.spheres{1,13}.center; 
% CP1 = [C1; C2; C3; C4];
% 
% Cs1 = transformed.spheres{1,16}.center; 
% Cs2 = transformed.spheres{1,15}.center; 
% Cs3 = transformed.spheres{1,14}.center; 
% Cs4 = transformed.spheres{1,13}.center; 
% CP2 = [Cs1; Cs2; Cs3; Cs4];
% 
% figure()
% hold on
% axis equal
% axis off
% plot3(CP1(1:3,1),CP1(1:3,2),CP1(1:3,3),'*-k')
% plot3(mesh.vertices(:,1),mesh.vertices(:,2),mesh.vertices(:,3),'.k')
% hold off
% 
% figure()
% hold on
% axis equal
% axis off
% plot3(CP2(1:3,1),CP2(1:3,2),CP2(1:3,3),'*R')
% plot3(transformed.vertices(:,1),transformed.vertices(:,2),transformed.vertices(:,3),'.R')
% hold off

end 



% % D2 re-scale
% J_cor_t = LMt(34:45,:);
% J_cor_s = LMs(34:45,:);
% 
% C1 = J_cor_t(12,:);
% C2 = J_cor_t(11,:);
% C3 = J_cor_t(10,:);
% CP1 = [C1; C2; C3];
% 
% Cs1 = J_cor_s(12,:);
% Cs2 = J_cor_s(11,:);
% Cs3 = J_cor_s(10,:);
% CPs1 = [Cs1; Cs2; Cs3];
% 
% keep = ismember(mesh.assignment, 9:11);
% [vertices, faces] = filter_vertices(mesh.vertices, mesh.faces, keep);
% 
% % input = V F CP1 CPs1 
% [vertices_n, CP2] = segment_scale_fingers(vertices, faces, CP1, CPs1);
% 
% clear C1 C2 C3 CP1 CP2 CP1 CPs1 CP2 Cs1 Cs2 Cs3 J_cor J_cor_s scale_factor1 scale_factor2 target_length1 target_length2
% clear vector1 vector1_n vector1s vector2 vector2_n vector2s C2_n C3_n Diff b bc W vertices faces normals keep J_cor_t