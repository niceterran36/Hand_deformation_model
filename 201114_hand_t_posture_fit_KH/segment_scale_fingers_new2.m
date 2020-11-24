function [mesh_tr] = segment_scale_fingers_new2(mesh, A, B)
% A: coordinates of template's CoR (joint1~16)
% B: coordinates of scan's CoR (joint1~16)

% D5_tip_t = A(1,:);
% D5_J_DIP_t = A(2,:);
% D5_J_PIP_t = A(3,:);
% D5_J_MCP_t = A(4,:);
% 
% D4_tip_t = A(5,:);
% D4_J_DIP_t = A(6,:);
% D4_J_PIP_t = A(7,:);
% D4_J_MCP_t = A(8,:);
% 
% D3_tip_t = A(9,:);
% D3_J_DIP_t = A(10,:);
% D3_J_PIP_t = A(11,:);
% D3_J_MCP_t = A(12,:);
% 
% D2_tip_t = A(13,:);
% D2_J_DIP_t = A(14,:);
% D2_J_PIP_t = A(15,:);
% D2_J_MCP_t = A(16,:);
% 
% D5_tip_s = B(1,:);
% D5_J_DIP_s = B(2,:);
% D5_J_PIP_s = B(3,:);
% D5_J_MCP_s = B(4,:);
% 
% D4_tip_s = B(5,:);
% D4_J_DIP_s = B(6,:);
% D4_J_PIP_s = B(7,:);
% D4_J_MCP_s = B(8,:);
% 
% D3_tip_s = B(9,:);
% D3_J_DIP_s = B(10,:);
% D3_J_PIP_s = B(11,:);
% D3_J_MCP_s = B(12,:);
% 
% D2_tip_s = B(13,:);
% D2_J_DIP_s = B(14,:);
% D2_J_PIP_s = B(15,:);
% D2_J_MCP_s = B(16,:);

compare_link = zeros(12,2);
compare_link(1,1) = norm(A(1,:)-A(2,:)); % template
compare_link(1,2) = norm(B(1,:)-B(2,:)); % target scan
compare_link(2,1) = norm(A(2,:)-A(3,:)); 
compare_link(2,2) = norm(B(2,:)-B(3,:)); 
compare_link(3,1) = norm(A(3,:)-A(4,:)); 
compare_link(3,2) = norm(B(3,:)-A(4,:));

compare_link(4,1) = norm(A(5,:)-A(6,:)); 
compare_link(4,2) = norm(B(5,:)-B(6,:)); 
compare_link(5,1) = norm(A(6,:)-A(7,:)); 
compare_link(5,2) = norm(B(6,:)-B(7,:)); 
compare_link(6,1) = norm(A(7,:)-A(8,:)); 
compare_link(6,2) = norm(B(7,:)-A(8,:));

compare_link(7,1) = norm(A(9,:)-A(10,:)); 
compare_link(7,2) = norm(B(9,:)-B(10,:)); 
compare_link(8,1) = norm(A(10,:)-A(11,:)); 
compare_link(8,2) = norm(B(10,:)-B(11,:)); 
compare_link(9,1) = norm(A(11,:)-A(12,:)); 
compare_link(9,2) = norm(B(11,:)-A(12,:));

compare_link(10,1) = norm(A(13,:)-A(14,:)); 
compare_link(10,2) = norm(B(13,:)-B(14,:)); 
compare_link(11,1) = norm(A(14,:)-A(15,:)); 
compare_link(11,2) = norm(B(14,:)-B(15,:)); 
compare_link(12,1) = norm(A(15,:)-A(16,:)); 
compare_link(12,2) = norm(B(15,:)-A(16,:));

compare_factor = compare_link(:,2)./compare_link(:,1);

%D2 scale

C1 = A(16,:); 
C2 = A(15,:);
C3 = A(14,:);
C4 = A(13,:);
CP1 = [C1; C2; C3; C4];

vector1 = CP1(2,:) - CP1(1,:);
vector2 = CP1(3,:) - CP1(2,:);
vector3 = CP1(4,:) - CP1(3,:);

vector1_n = compare_factor(12)*vector1;
vector2_n = compare_factor(11)*vector2;
vector3_n = compare_factor(10)*vector3;

C2_n = CP1(1,:) + vector1_n;
C3_n = C2_n + vector2_n;
C4_n = C3_n + vector3_n;
CP2 = [CP1(1,:); C2_n; C3_n; C4_n];

keep = ismember(mesh.assignment, 9:11);
[vertices, faces] = filter_vertices(mesh.vertices, mesh.faces, keep);

[b,bc] = bbw_boundary_conditions(vertices, faces, CP1(1:4,:));
W = bbw_biharmonic_bounded(vertices, faces, b, bc);
W = W./repmat(sum(W,2),1,size(W,2));
Diff = CP2(1:4,:) - CP1(1:4,:);
V_tr = bbw_simple_deform(vertices, faces, CP1(1:4,:), W, Diff);

mesh.vertices(keep,:) = V_tr;
mesh.spheres{1,3}.center = C2_n; 
mesh.spheres{1,2}.center = C3_n;
mesh.spheres{1,1}.center = C4_n;

A(15,:) = C2_n; 
A(14,:) = C3_n;
A(13,:) = C4_n;

clear vector1 vector2 vector3 vector1_n vector2_n vector3_n C2_n C3_n C4_n C1 C2 C3 C4 CP1 CP2
clear b bc C1 C2 C2_n C3 C3_n C4 C4_n CP1 CP2 Diff V_tr vector1 vector1_n vector2 vector2_n W vertices faces keep

% D3 scale
C1 = A(12,:); 
C2 = A(11,:); 
C3 = A(10,:); 
C4 = A(9,:); 
CP1 = [C1; C2; C3; C4];

vector1 = CP1(2,:) - CP1(1,:);
vector2 = CP1(3,:) - CP1(2,:);
vector3 = CP1(4,:) - CP1(3,:);

vector1_n = compare_factor(9)*vector1;
vector2_n = compare_factor(8)*vector2;
vector3_n = compare_factor(7)*vector3;

C2_n = CP1(1,:) + vector1_n;
C3_n = C2_n + vector2_n;
C4_n = C3_n + vector3_n;
CP2 = [CP1(1,:); C2_n; C3_n; C4_n];

keep = ismember(mesh.assignment, 12:14);
[vertices, faces] = filter_vertices(mesh.vertices, mesh.faces, keep);

[b,bc] = bbw_boundary_conditions(vertices, faces, CP1(1:4,:));
W = bbw_biharmonic_bounded(vertices, faces, b, bc);
W = W./repmat(sum(W,2),1,size(W,2));
Diff = CP2(1:4,:) - CP1(1:4,:);
V_tr = bbw_simple_deform(vertices, faces, CP1(1:4,:), W, Diff);

mesh.vertices(keep,:) = V_tr;
mesh.spheres{1,11}.center = C2_n; 
mesh.spheres{1,10}.center = C3_n;
mesh.spheres{1,9}.center = C4_n;

A(11,:) = C2_n; 
A(10,:) = C3_n;
A(9,:) = C4_n;

clear vector1 vector2 vector3 vector1_n vector2_n vector3_n C2_n C3_n C4_n C1 C2 C3 C4 CP1 CP2
clear b bc C1 C2 C2_n C3 C3_n C4 C4_n CP1 CP2 Diff V_tr vector1 vector1_n vector2 vector2_n W vertices faces keep

% D4 scale
C1 = A(8,:);  
C2 = A(7,:);  
C3 = A(6,:);  
C4 = A(5,:);  
CP1 = [C1; C2; C3; C4];

vector1 = CP1(2,:) - CP1(1,:);
vector2 = CP1(3,:) - CP1(2,:);
vector3 = CP1(4,:) - CP1(3,:);

vector1_n = compare_factor(6)*vector1;
vector2_n = compare_factor(5)*vector2;
vector3_n = compare_factor(4)*vector3;

C2_n = CP1(1,:) + vector1_n;
C3_n = C2_n + vector2_n;
C4_n = C3_n + vector3_n;
CP2 = [CP1(1,:); C2_n; C3_n; C4_n];


keep = ismember(mesh.assignment, 15:17);
[vertices, faces] = filter_vertices(mesh.vertices, mesh.faces, keep);

[b,bc] = bbw_boundary_conditions(vertices, faces, CP1(1:4,:));
W = bbw_biharmonic_bounded(vertices, faces, b, bc);
W = W./repmat(sum(W,2),1,size(W,2));
Diff = CP2(1:4,:) - CP1(1:4,:);
V_tr = bbw_simple_deform(vertices, faces, CP1(1:4,:), W, Diff);

mesh.vertices(keep,:) = V_tr;
mesh.spheres{1,7}.center = C2_n; 
mesh.spheres{1,6}.center = C3_n;
mesh.spheres{1,5}.center = C4_n;

A(7,:) = C2_n; 
A(6,:) = C3_n;
A(5,:) = C4_n;

clear vector1 vector2 vector3 vector1_n vector2_n vector3_n C2_n C3_n C4_n C1 C2 C3 C4 CP1 CP2
clear b bc C1 C2 C2_n C3 C3_n C4 C4_n CP1 CP2 Diff V_tr vector1 vector1_n vector2 vector2_n W vertices faces keep

% D5 scale

C1 = A(4,:);
C2 = A(3,:);
C3 = A(2,:);
C4 = A(1,:); 
CP1 = [C1; C2; C3; C4];

vector1 = CP1(2,:) - CP1(1,:);
vector2 = CP1(3,:) - CP1(2,:);
vector3 = CP1(4,:) - CP1(3,:);

vector1_n = compare_factor(3)*vector1;
vector2_n = compare_factor(2)*vector2;
vector3_n = compare_factor(1)*vector3;

C2_n = CP1(1,:) + vector1_n;
C3_n = C2_n + vector2_n;
C4_n = C3_n + vector3_n;
CP2 = [CP1(1,:); C2_n; C3_n; C4_n];

keep = ismember(mesh.assignment, 18:20);
[vertices, faces] = filter_vertices(mesh.vertices, mesh.faces, keep);

[b,bc] = bbw_boundary_conditions(vertices, faces, CP1(1:4,:));
W = bbw_biharmonic_bounded(vertices, faces, b, bc);
W = W./repmat(sum(W,2),1,size(W,2));
Diff = CP2(1:4,:) - CP1(1:4,:);
V_tr = bbw_simple_deform(vertices, faces, CP1(1:4,:), W, Diff);

mesh.vertices(keep,:) = V_tr;
mesh.spheres{1,3}.center = C2_n; 
mesh.spheres{1,2}.center = C3_n;
mesh.spheres{1,1}.center = C4_n;

A(3,:) = C2_n; 
A(2,:) = C3_n;
A(1,:) = C4_n;

clear vector1 vector2 vector3 vector1_n vector2_n vector3_n C2_n C3_n C4_n C1 C2 C3 C4 CP1 CP2
clear b bc C1 C2 C2_n C3 C3_n C4 C4_n CP1 CP2 Diff V_tr vector1 vector1_n vector2 vector2_n W vertices faces keep

mesh_tr = mesh;

end 
% 
% A_tr = A;

% Digit 5 re-scale

% compare_link = zeros(12,2);
% compare_link(1,1) = norm(A(1,:)-A(2,:)); % template
% compare_link(1,2) = norm(B(1,:)-B(2,:)); % target scan
% compare_link(2,1) = norm(A(2,:)-A(3,:)); 
% compare_link(2,2) = norm(B(2,:)-B(3,:)); 
% compare_link(3,1) = norm(A(3,:)-A(4,:)); 
% compare_link(3,2) = norm(B(3,:)-A(4,:)); 
% compare_factor = compare_link(:,2)./compare_link(:,1);
% 
% keep = ismember(mesh.assignment, 18:20);
% [vertices, faces] = filter_vertices(mesh.vertices, mesh.faces, keep);
% 
% t_vector1 = D5_J_PIP_t - D5_J_MCP_t;
% t_vector2 = D5_J_DIP_t - D5_J_PIP_t;
% t_vector3 = D5_tip_t - D5_J_DIP_t;
% s_vector1 = D5_J_PIP_s - D5_J_MCP_t;
% s_vector2 = D5_J_DIP_s - D5_J_PIP_s;
% s_vector3 = D5_tip_s - D5_J_DIP_s;
% 
% scale_factor1 = norm(s_vector1)/norm(t_vector1);
% scale_factor2 = norm(s_vector2)/norm(t_vector2);
% scale_factor3 = norm(s_vector3)/norm(t_vector3);
% 
% clear t_vector1 t_vector2 t_vector3 s_vector1 s_vector2 s_vector3
% 
% C1 = mesh.spheres{1,4}.center; 
% C2 = mesh.spheres{1,3}.center; 
% C3 = mesh.spheres{1,2}.center; 
% C4 = mesh.spheres{1,1}.center; 
% CP1 = [C1; C2; C3; C4];
% 
% vector1 = CP1(2,:) - CP1(1,:);
% vector2 = CP1(3,:) - CP1(2,:);
% vector3 = CP1(4,:) - CP1(3,:);
% 
% vector1_n = scale_factor1*vector1;
% vector2_n = scale_factor2*vector2;
% vector3_n = scale_factor3*vector3;
% 
% C2_n = CP1(1,:) + vector1_n;
% C3_n = C2_n + vector2_n;
% C4_n = C3_n + vector3_n;
% CP2 = [CP1(1,:); C2_n; C3_n; C4_n];
% 
% [b,bc] = bbw_boundary_conditions(vertices, faces, CP1(1:4,:));
% W = bbw_biharmonic_bounded(vertices, faces, b, bc);
% W = W./repmat(sum(W,2),1,size(W,2));
% Diff = CP2(1:4,:) - CP1(1:4,:);
% V_tr = bbw_simple_deform(vertices, faces, CP1(1:4,:), W, Diff);
% 
% mesh.vertices(keep,:) = V_tr;
% mesh.spheres{1,3}.center = C2_n; 
% mesh.spheres{1,2}.center = C3_n;
% mesh.spheres{1,1}.center = C4_n;
% 
% clear b bc C1 C2 C2_n C3 C3_n C4 C4_n CP1 CP2 Diff V_tr vector1 vector1_n vector2 vector2_n W vertices faces keep
% 
% % Digit 2 re-scale
% keep = ismember(mesh.assignment, 9:11);
% [vertices, faces] = filter_vertices(mesh.vertices, mesh.faces, keep);
% 
% t_vector1 = D2_J_PIP_t - D2_J_MCP_t;
% t_vector2 = D2_J_DIP_t - D2_J_PIP_t;
% t_vector3 = D2_tip_t - D2_J_DIP_t;
% s_vector1 = D2_J_PIP_s - D2_J_MCP_t;
% s_vector2 = D2_J_DIP_s - D2_J_PIP_s;
% s_vector3 = D2_tip_s - D2_J_DIP_s;
% 
% scale_factor1 = norm(s_vector1)/norm(t_vector1);
% scale_factor2 = norm(s_vector2)/norm(t_vector2);
% scale_factor3 = norm(s_vector3)/norm(t_vector3);
% 
% clear t_vector1 t_vector2 t_vector3 s_vector3 s_vector1 s_vector2
% 
% C1 = mesh.spheres{1,16}.center; 
% C2 = mesh.spheres{1,15}.center; 
% C3 = mesh.spheres{1,14}.center; 
% C4 = mesh.spheres{1,13}.center; 
% CP1 = [C1; C2; C3; C4];
% 
% vector1 = CP1(2,:) - CP1(1,:);
% vector2 = CP1(3,:) - CP1(2,:);
% vector3 = CP1(4,:) - CP1(3,:);
% 
% vector1_n = scale_factor1*vector1;
% vector2_n = scale_factor2*vector2;
% vector3_n = scale_factor3*vector3;
% 
% C2_n = CP1(1,:) + vector1_n;
% C3_n = C2_n + vector2_n;
% C4_n = C3_n + vector3_n;
% CP2 = [CP1(1,:); C2_n; C3_n; C4_n];
% 
% [b,bc] = bbw_boundary_conditions(vertices, faces, CP1(1:4,:));
% W = bbw_biharmonic_bounded(vertices, faces, b, bc);
% W = W./repmat(sum(W,2),1,size(W,2));
% Diff = CP2(1:4,:) - CP1(1:4,:);
% V_tr = bbw_simple_deform(vertices, faces, CP1(1:4,:), W, Diff);
% 
% mesh.vertices(keep,:) = V_tr;
% mesh.spheres{1,15}.center = C2_n; 
% mesh.spheres{1,14}.center = C3_n;
% mesh.spheres{1,13}.center = C4_n;
% 
% clear b bc C1 C2 C2_n C3 C3_n C4 C4_n CP1 CP2 Diff V_tr vector1 vector1_n vector2 vector2_n W vertices faces keep
% 
% % Digit 3 re-scale
% keep = ismember(mesh.assignment, 12:14);
% [vertices, faces] = filter_vertices(mesh.vertices, mesh.faces, keep);
% 
% t_vector1 = D3_J_PIP_t - D3_J_MCP_t;
% t_vector2 = D3_J_DIP_t - D3_J_PIP_t;
% t_vector3 = D3_tip_t - D3_J_DIP_t;
% 
% s_vector1 = D3_J_PIP_s - D3_J_MCP_t;
% s_vector2 = D3_J_DIP_s - D3_J_PIP_s;
% s_vector3 = D3_tip_s - D3_J_DIP_s;
% 
% scale_factor1 = norm(s_vector1)/norm(t_vector1);
% scale_factor2 = norm(s_vector2)/norm(t_vector2);
% scale_factor3 = norm(s_vector3)/norm(t_vector3);
% 
% clear t_vector1 t_vector2 t_vector3 s_vector3 s_vector1 s_vector2
% 
% C1 = mesh.spheres{1,12}.center; 
% C2 = mesh.spheres{1,11}.center; 
% C3 = mesh.spheres{1,10}.center; 
% C4 = mesh.spheres{1,9}.center; 
% CP1 = [C1; C2; C3; C4];
% 
% vector1 = CP1(2,:) - CP1(1,:);
% vector2 = CP1(3,:) - CP1(2,:);
% vector3 = CP1(4,:) - CP1(3,:);
% 
% vector1_n = scale_factor1*vector1;
% vector2_n = scale_factor2*vector2;
% vector3_n = scale_factor3*vector3;
% 
% C2_n = CP1(1,:) + vector1_n;
% C3_n = C2_n + vector2_n;
% C4_n = C3_n + vector3_n;
% CP2 = [CP1(1,:); C2_n; C3_n; C4_n];
% 
% [b,bc] = bbw_boundary_conditions(vertices, faces, CP1(1:4,:));
% W = bbw_biharmonic_bounded(vertices, faces, b, bc);
% W = W./repmat(sum(W,2),1,size(W,2));
% Diff = CP2(1:4,:) - CP1(1:4,:);
% V_tr = bbw_simple_deform(vertices, faces, CP1(1:4,:), W, Diff);
% 
% mesh.vertices(keep,:) = V_tr;
% mesh.spheres{1,11}.center = C2_n; 
% mesh.spheres{1,10}.center = C3_n;
% mesh.spheres{1,9}.center = C4_n;
% 
% clear b bc C1 C2 C2_n C3 C3_n C4 C4_n CP1 CP2 Diff V_tr vector1 vector1_n vector2 vector2_n W vertices faces keep
% 
% % Digit 4 re-scale
% keep = ismember(mesh.assignment, 15:17);
% [vertices, faces] = filter_vertices(mesh.vertices, mesh.faces, keep);
% 
% t_vector1 = D4_J_PIP_t - D4_J_MCP_t;
% t_vector2 = D4_J_DIP_t - D4_J_PIP_t;
% t_vector3 = D4_tip_t - D4_J_DIP_t;
% s_vector1 = D4_J_PIP_s - D4_J_MCP_t;
% s_vector2 = D4_J_DIP_s - D4_J_PIP_s;
% s_vector3 = D4_tip_s - D4_J_DIP_s;
% 
% scale_factor1 = norm(s_vector1)/norm(t_vector1);
% scale_factor2 = norm(s_vector2)/norm(t_vector2);
% scale_factor3 = norm(s_vector3)/norm(t_vector3);
% 
% clear t_vector1 t_vector2 t_vector3 s_vector1 s_vector2 s_vector3
% 
% C1 = mesh.spheres{1,8}.center; 
% C2 = mesh.spheres{1,7}.center; 
% C3 = mesh.spheres{1,6}.center; 
% C4 = mesh.spheres{1,5}.center; 
% CP1 = [C1; C2; C3; C4];
% 
% vector1 = CP1(2,:) - CP1(1,:);
% vector2 = CP1(3,:) - CP1(2,:);
% vector3 = CP1(4,:) - CP1(3,:);
% 
% vector1_n = scale_factor1*vector1;
% vector2_n = scale_factor2*vector2;
% vector3_n = scale_factor3*vector3;
% 
% C2_n = CP1(1,:) + vector1_n;
% C3_n = C2_n + vector2_n;
% C4_n = C3_n + vector3_n;
% CP2 = [CP1(1,:); C2_n; C3_n; C4_n];
% 
% [b,bc] = bbw_boundary_conditions(vertices, faces, CP1(1:4,:));
% W = bbw_biharmonic_bounded(vertices, faces, b, bc);
% W = W./repmat(sum(W,2),1,size(W,2));
% Diff = CP2(1:4,:) - CP1(1:4,:);
% V_tr = bbw_simple_deform(vertices, faces, CP1(1:4,:), W, Diff);
% 
% mesh.vertices(keep,:) = V_tr;
% mesh.spheres{1,7}.center = C2_n; 
% mesh.spheres{1,6}.center = C3_n;
% mesh.spheres{1,5}.center = C4_n;
% 
% clear b bc C1 C2 C2_n C3 C3_n C4 C4_n CP1 CP2 Diff V_tr vector1 vector1_n vector2 vector2_n W vertices faces keep


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


