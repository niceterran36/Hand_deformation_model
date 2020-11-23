%%
centers = zeros(30,3);
for i = 1:30
centers(i,:) = mesh.spheres{1,i}.center;
end

JB_cor = [];
JB_pos1 = zeros(17,3);
JB_pos1(17,1:3) = [-5.2816 -69.8655 103.169];
JB_pos1(14:16,:) = JB_cor(1:3,1:3);
JB_pos1(10:12,:) = JB_cor(1:3,4:6);
JB_pos1(6:8,:) = JB_cor(1:3,7:9);
JB_pos1(2:4,:) = JB_cor(1:3,10:12);

A = centers(1:22,:); % template's CoR
B = JB_pos1; % Scan's CoR

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
plot3(B(2:4,1),B(2:4,2),B(2:4,3),'-r');
plot3(B(6:8,1),B(6:8,2),B(6:8,3),'-r');
plot3(B(10:12,1),B(10:12,2),B(10:12,3),'-r');
plot3(B(14:16,1),B(14:16,2),B(14:16,3),'-r');
plot3(B([4 17 8],1),B([4 17 8],2),B([4 17 8],3),'-r')
plot3(B([12 17 16],1),B([12 17 16],2),B([12 17 16],3),'-r')
hold off

%% scale
compare_factor = compare_factor_cal(A,B); % tip location add & update  ==> current tip scale = 1
A_tr = segment_scale_simple(A, compare_factor);
A = A_tr;

A(21,:) = [];
A(17:20,:) = [];
B(1,:) = A(1,:); % tip location add & update 
B(5,:) = A(5,:); % tip location add & update
B(9,:) = A(9,:); % tip location add & update
B(13,:) = A(13,:); % tip location add & update

[transformed] = segment_scale_fingers_new2(mesh, A, B);
mesh = transformed;
clear transformed

% figure()
% axis equal
% axis off
% hold on
% plot3(mesh.vertices(:,1), mesh.vertices(:,2), mesh.vertices(:,3),'.k')
% plot3(V_tr(:,1), V_tr(:,2), V_tr(:,3),'.c')
% hold off

%%

axes = bone_axes(mesh.spheres);

% call MCP axes

org = axes{6}(1:3,4)';

A_tr_o = A - org;
B_tr_o = B - org;

ax1 = axes{6}(1:3,1)';
ax2 = axes{6}(1:3,2)';
ax3 = axes{6}(1:3,3)';
%% ======================================== start here %https://mrw0119.tistory.com/94


angle = zeros(19,1);
tr_mesh = transform_angle(mesh, angle);




load('Ags.mat');
angle = Ags(:,5);
tr_mesh = transform_angle(mesh, angle);
mesh = tr_mesh;


angle = zeros(19,1);

transforms = cell(1, 18);
for i = 1 : 18
    transforms{i} = eye(4);
end
axes = bone_axes(mesh.spheres);

transforms_ad = cell(1,4);
for i = 1 : 4
    transforms_ad{i} = eye(4);
end 

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

transformed = skin_dualquat(mesh, transforms);
for i = 1:18
     transformed.centers(i,:) = transformed.spheres{1,i}.center;
end





























