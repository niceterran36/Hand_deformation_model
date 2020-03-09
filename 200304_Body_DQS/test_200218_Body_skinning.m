clc 
clear all
addpath('C:\Users\Hayoung Jung\Documents\[GitHub-Labtop]\Hand_deformation_model\functions');
addpath(genpath('external'));
%addpath(genpath('functions'));
%addpath('F:\[GitHub]\Hand_deformation_model\functions');
%addpath('/Users/user/Documents/GitHub/Hand_deformation_model\functions');

load('Body_temp.mat');
load('mesh_repose.mat');
Body_temp = mesh_repose;

V = Body_temp.V; F = Body_temp.F; COR = Body_temp.COR; v_segment = Body_temp.v_segment; weights = Body_temp.weights;
Body_temp.parent = Body_temp.COR_bone(:,2); 
C = COR;

for i = 1:length(COR)
Body_temp.spheres{i}.center = COR(i,:);
Body_temp.spheres{i}.bone = Body_temp.parent(i);
Body_temp.bones{i}.parent = Body_temp.parent(i);
end 

%% Visualization 3D body
C = COR;
figure()
hold on
axis equal
h = trimesh(F, V(:,1), V(:,2), V(:,3),'EdgeColor','none','FaceColor',[0.65, 0.65, 0.65],'FaceAlpha', 1);
lighting gouraud;
view([-90, 0]);
camlight;
view([90, 0]);
camlight;

%scatter3(V(:,1),V(:,2),V(:,3),'.', 'MarkerEdgeColor',[180/255, 180/255, 180/255])
plot3(C(:,1),C(:,2),C(:,3),'b*')
plot3(C([1 10 11 12 13],1),C([1 10 11 12 13],2),C([1 10 11 12 13],3), 'k-');
plot3(C(1:5,1),C(1:5,2),C(1:5,3), 'k-');
plot3(C([1 6:9],1),C([1 6:9],2),C([1 6:9],3), 'k-');
plot3(C(14:17,1),C(14:17,2),C(14:17,3), 'k-');
plot3(C(18:21,1),C(18:21,2),C(18:21,3), 'k-');
plot3(C([14 11 18],1),C([14 11 18],2),C([14 11 18],3), 'k-');
hold off

%% Transform matrix setting

transforms = cell(1, 21);
for i = 1 : 21
    transforms{i} = eye(4);
end
axes = bone_axes_body(COR);

% left-arm setting
% COR_leftarm_link = zeros(16,1);
% COR_leftarm_link(14:16,1) = [15; 16; 17];

%% Previous transformation 
angle = zeros(4,1);

% 1 rad = 57.3 deg. 0.785 rad = 45.0 deg.
% left arm 
angle(1) = 0/57.3; 
angle(2) = 0/57.3; 
angle(3) = 30/57.3; 
angle(4) = 0/57.3;

% Shoulder axis1: flexion(-)/extension(+), axis2: abduction(+)/adduction(-), axis3: supination/pronation
% Elbow axis2: flexion/extension
% Wrist axis1: ulnar(-)/radial(+), axis2: flexion(-)/extension(+), axis3: supination/pronation


% left arm 
transforms{14} = matrix_rotation( ... % left shoulder
    angle(1), ...  
    matrix_apply(transforms{11}, axes{14}(1 : 3, 1)'), ... 
    matrix_apply(transforms{11}, axes{14}(1 : 3, 4)') ... % center
) * transforms{11};
transforms{15} = matrix_rotation( ... % left elbow
    angle(2), ...
    matrix_apply(transforms{14}, axes{15}(1 : 3, 2)'), ...
    matrix_apply(transforms{14}, axes{15}(1 : 3, 4)') ...
) * transforms{14};
transforms{16} = matrix_rotation( ... % left wrist
    angle(3), ...
    matrix_apply(transforms{15}, axes{16}(1 : 3, 2)'), ...
    matrix_apply(transforms{15}, axes{16}(1 : 3, 4)') ...
) * transforms{15};

    
for i = 2:21
    COR(i,:) = matrix_apply(transforms{i-1}, COR(i,:));
end 
axes = bone_axes_body(COR);

    
    
% % right arm
% transforms{18} = matrix_rotation( ... % right shoulder
%     angle(4), ... % rotation angle: 0 ~ 2, range 3 = 180 deg.  
%     matrix_apply(transforms{11}, axes{18}(1 : 3, 3)'), ... % axis1: axis2: flexion(-)/extention(+); axis3: abduction(+)/adduction(-)
%     matrix_apply(transforms{11}, axes{18}(1 : 3, 4)') ... % center
% ) * transforms{11};
% transforms{19} = matrix_rotation( ... % left elbow
%     angle(5), ...
%     matrix_apply(transforms{18}, axes{19}(1 : 3, 2)'), ...
%     matrix_apply(transforms{18}, axes{19}(1 : 3, 4)') ...
% ) * transforms{18};
% transforms{20} = matrix_rotation( ... % left wrist
%     angle(6), ...
%     matrix_apply(transforms{19}, axes{20}(1 : 3, 2)'), ...
%     matrix_apply(transforms{19}, axes{20}(1 : 3, 4)') ...
% ) * transforms{19};
% 
% % left leg
% transforms{2} = matrix_rotation( ... % right shoulder
%     angle(4), ... % rotation angle: 0 ~ 2, range 3 = 180 deg.  
%     matrix_apply(transforms{1}, axes{2}(1 : 3, 3)'), ... % axis1: axis2: flexion(-)/extention(+); axis3: abduction(+)/adduction(-)
%     matrix_apply(transforms{1}, axes{2}(1 : 3, 4)') ... % center
% ) * transforms{1};
% transforms{3} = matrix_rotation( ... % left elbow
%     angle(5), ...
%     matrix_apply(transforms{2}, axes{3}(1 : 3, 2)'), ...
%     matrix_apply(transforms{2}, axes{3}(1 : 3, 4)') ...
% ) * transforms{2};
% transforms{4} = matrix_rotation( ... % left wrist
%     angle(6), ...
%     matrix_apply(transforms{3}, axes{4}(1 : 3, 2)'), ...
%     matrix_applytransforms{3}, axes{4}(1 : 3, 4)') ...
% ) * transforms{3};




%% Deformation apply

transformed = Body_temp;
transformed = skin_dualquat_body(transformed, transforms);
COR_tr = COR;

%% CoR transformation

% COR_tr(15,:) = matrix_apply(transforms{14}, COR_tr(15,:));
% COR_tr(16,:) = matrix_apply(transforms{15}, COR_tr(16,:));
% COR_tr(17,:) = matrix_apply(transforms{16}, COR_tr(17,:));

% for i = 16:17
%     COR_tr(i,:) = matrix_apply(transforms{i-1}, COR_tr(i,:));
% end 


%% CoR Transformed plot
figure()
hold on
axis equal
h = trimesh(transformed.F, transformed.V(:,1), transformed.V(:,2), transformed.V(:,3),'EdgeColor','none','FaceColor',[0.65, 0.65, 0.65],'FaceAlpha', 1);
lighting gouraud;
view([-90, 0]);
camlight;
view([90, 0]);
camlight;
%scatter3(COR(14,1),COR(14,2),COR(14,3),'*', 'MarkerEdgeColor',[0/255, 0/255, 0/255])
%scatter3(COR_tr(:,1),COR_tr(:,2),COR_tr(:,3),'*', 'MarkerEdgeColor',[0/255, 0/255, 255/255])
plot3(COR_tr([1 10 11 12 13],1),COR_tr([1 10 11 12 13],2),COR_tr([1 10 11 12 13],3), 'k-');
plot3(COR_tr(1:5,1),COR_tr(1:5,2),COR_tr(1:5,3), 'k-');
plot3(COR_tr([1 6:9],1),COR_tr([1 6:9],2),COR_tr([1 6:9],3), 'k-');
plot3(COR_tr(14:17,1),COR_tr(14:17,2),COR_tr(14:17,3), 'k-');
plot3(COR_tr(18:21,1),COR_tr(18:21,2),COR_tr(18:21,3), 'k-');
plot3(COR_tr([14 11 18],1),COR_tr([14 11 18],2),COR_tr([14 11 18],3), 'k-');
hold off
%% Save current template 

transformed.COR = COR;
mesh_repose = transformed;
mesh_repose.V = V;
mesh_repose.COR = COR;
save mesh_repose.mat mesh_repose;

figure()
hold on
axis equal
h = trimesh(mesh_repose.F, mesh_repose.V(:,1), mesh_repose.V(:,2), mesh_repose.V(:,3),'EdgeColor','none','FaceColor',[0.65, 0.65, 0.65],'FaceAlpha', 1);
lighting gouraud;
view([-90, 0]);
camlight;
view([90, 0]);
camlight;
hold off

%% after transformation

C = COR_tr;
V = transformed.V;
figure()
hold on
axis equal
scatter3(V(:,1),V(:,2),V(:,3),'.', 'MarkerEdgeColor',[180/255, 180/255, 180/255])
plot3(C(:,1),C(:,2),C(:,3),'b*')
plot3(C([1 10 11 12 13],1),C([1 10 11 12 13],2),C([1 10 11 12 13],3), 'k-');
plot3(C(1:5,1),C(1:5,2),C(1:5,3), 'k-');
plot3(C([1 6:9],1),C([1 6:9],2),C([1 6:9],3), 'k-');
plot3(C(14:17,1),C(14:17,2),C(14:17,3), 'k-');
plot3(C(18:21,1),C(18:21,2),C(18:21,3), 'k-');
plot3(C([14 11 18],1),C([14 11 18],2),C([14 11 18],3), 'k-');
hold off

%% angle apply
angle = zeros(19,1);

angle(1) = -0.4;
angle(2) = 0.2;
angle(3) = -0.6;

angle(4) = -0.15;
angle(5) = -0.35;
angle(6) = -0.1;

angle(7) = -0.3;
angle(8) = -0.25;
angle(9) = -0.15;

angle(10) = -0.35;
angle(11) = -0.40;
angle(12) = -0.1;

angle(13) = -0.65;
angle(14) = -0.4;
angle(15) = 0;

%MCP
% angle(16) = ;
angle(17) = 0.1;
angle(18) = 0.1;
% angle(19) = ;

%% MCP abduction/adduction motion
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

%% Flexion/extension of fingers 
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
