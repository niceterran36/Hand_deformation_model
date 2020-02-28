clc 
clear all
addpath('F:\[GitHub]\Hand_deformation_model\functions');
addpath(genpath('external'));
%addpath('C:\Users\Hayoung Jung\Documents\[GitHub-Labtop]\Hand_deformation_model\functions');

load('Body_temp.mat');
V = Body_temp.V; F = Body_temp.F; COR = Body_temp.COR; v_segment = Body_temp.v_segment; weights = Body_temp.weights;
C = COR;

%% Visualization 3D body
C = COR;
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

%% Transform matrix setting

transforms = cell(1, 21);
for i = 1 : 21
    transforms{i} = eye(4);
end
axes = bone_axes_body(COR);

% left-arm setting
COR_leftarm_link = zeros(16,1);
COR_leftarm_link(14:16,1) = [15; 16; 17];

%% transform matrix per vertex

% tansforms_vi = cell(1,13249);
% for i = 1:13249
%     transforms_vi{1} = eye(4);
% end
% axes = bone_axes_body(COR);

%%
angle = zeros(4,1);
COR_tr = COR;

% 1 rad = 57.3 deg. 0.785 rad = 45.0 deg.
angle(1) = 0;
%angle(2) = -1; shoulder flexion/extension
angle(2) = 0;
angle(3) = 0; 
angle(4) = 0;

transforms{11} = matrix_rotation(...
    angle(1), ... % rotation angle  
    matrix_apply(transforms{10}, axes{11}(1 : 3, 1)'), ... % axis1: XX; axis2: XX; axis3: XXe
    matrix_apply(transforms{10}, axes{11}(1 : 3, 4)') ... % center
) * transforms{10};
% transforms{14} = matrix_rotation( ... % left shoulder
%     angle(2), ... % rotation angle: 0 ~ 2, range 3 = 180 deg.  
%     matrix_apply(transforms{11}, axes{14}(1 : 3, 2)'), ... % axis1: axis2: flexion(-)/extention(+); axis3: abduction(+)/adduction(-)
%     matrix_apply(transforms{11}, axes{14}(1 : 3, 4)') ... % center
% ) * transforms{11};
transforms{14} = matrix_rotation( ... % left shoulder
    angle(2), ... % rotation angle: 0 ~ 2, range 3 = 180 deg.  
    matrix_apply(transforms{11}, axes{14}(1 : 3, 3)'), ... % axis1: axis2: flexion(-)/extention(+); axis3: abduction(+)/adduction(-)
    matrix_apply(transforms{11}, axes{14}(1 : 3, 4)') ... % center
) * transforms{11};

transforms{15} = matrix_rotation( ... % left elbow
    angle(3), ...
    matrix_apply(transforms{14}, axes{15}(1 : 3, 2)'), ...
    matrix_apply(transforms{14}, axes{15}(1 : 3, 4)') ...
) * transforms{14};
transforms{16} = matrix_rotation( ... % left wrist
    angle(4), ...
    matrix_apply(transforms{15}, axes{16}(1 : 3, 2)'), ...
    matrix_apply(transforms{15}, axes{16}(1 : 3, 4)') ...
) * transforms{15};
transforms{17} = matrix_rotation( ... % hand tip
    0, ...
    matrix_apply(transforms{16}, axes{16}(1 : 3, 2)'), ... 
    matrix_apply(transforms{16}, axes{16}(1 : 3, 4)') ...
) * transforms{16};

for i = 15:17
    COR_tr(i,:) = matrix_apply(transforms{i-1}, COR_tr(i,:));
end 
axes = bone_axes_body(COR_tr);

angle(3) = 1; 
transforms{15} = matrix_rotation( ... % left elbow
    angle(3), ...
    matrix_apply(transforms{14}, axes{15}(1 : 3, 2)'), ... % axis2: flexion(+)/extension(-)
    matrix_apply(transforms{14}, axes{15}(1 : 3, 4)') ...
) * transforms{14};
transforms{16} = matrix_rotation( ... % left wrist
    angle(4), ...
    matrix_apply(transforms{15}, axes{16}(1 : 3, 2)'), ... % axis2: ulnar/radial deviation
    matrix_apply(transforms{15}, axes{16}(1 : 3, 4)') ...
) * transforms{15};
transforms{17} = matrix_rotation( ... % hand tip
    0, ...
    matrix_apply(transforms{16}, axes{16}(1 : 3, 2)'), ...
    matrix_apply(transforms{16}, axes{16}(1 : 3, 4)') ...
) * transforms{16};

for i = 16:17
    COR_tr(i,:) = matrix_apply(transforms{i-1}, COR_tr(i,:));
end
axes = bone_axes_body(COR_tr);

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
%scatter3(COR(14,1),COR(14,2),COR(14,3),'*', 'MarkerEdgeColor',[0/255, 0/255, 0/255])
scatter3(COR_tr(:,1),COR_tr(:,2),COR_tr(:,3),'*', 'MarkerEdgeColor',[0/255, 0/255, 255/255])
plot3(COR_tr([1 10 11 12 13],1),COR_tr([1 10 11 12 13],2),COR_tr([1 10 11 12 13],3), 'k-');
plot3(COR_tr(1:5,1),COR_tr(1:5,2),COR_tr(1:5,3), 'k-');
plot3(COR_tr([1 6:9],1),COR_tr([1 6:9],2),COR_tr([1 6:9],3), 'k-');
plot3(COR_tr(14:17,1),COR_tr(14:17,2),COR_tr(14:17,3), 'k-');
plot3(COR_tr(18:21,1),COR_tr(18:21,2),COR_tr(18:21,3), 'k-');
plot3(COR_tr([14 11 18],1),COR_tr([14 11 18],2),COR_tr([14 11 18],3), 'k-');
hold off
%% new transformation function

weight14 = weights(:,14);
LI = weight14 ~= 0;  % V14 vertices
LII = weight14 == 0; % non_V14 vertices
V14 = V(LI,:);
non_V14 = V(LII,:);
V14_weights = weight14(LI);

weighted_tr = zeros(size(V,1),3);
for i = 1:size(V,1)
    basic_T = eye(4);
    weighted_T = transforms{14}*weight14(i);
    
    if weight14(i) == 0
        T = basic_T;
    else
        T = weighted_T;;
    end
    %transforms{14} - T
    weighted_tr(i,:) = matrix_apply(T, V(i,:),1);
end 

test = weighted_tr - V


% vectors = matrix_apply(matrix, vectors, constant)
V14_transformed = matrix_apply(transforms{14},V14,1);

figure()
hold on
axis equal
scatter3(COR(14,1),COR(14,2),COR(14,3),'*', 'MarkerEdgeColor',[0/255, 0/255, 0/255])
scatter3(V14(:,1),V14(:,2),V14(:,3),'.', 'MarkerEdgeColor',[255/255, 0/255, 0/255])
scatter3(non_V14(:,1),non_V14(:,2),non_V14(:,3),'.', 'MarkerEdgeColor',[180/255, 180/255, 180/255])
scatter3(V14_transformed(:,1),V14_transformed(:,2),V14_transformed(:,3),'.', 'MarkerEdgeColor',[0/255, 0/255, 255/255])
hold off

figure()
hold on
axis equal
scatter3(COR(14:15,1),COR(14:15,2),COR(14:15,3),'*', 'MarkerEdgeColor',[0/255, 0/255, 0/255])
% scatter3(V14(:,1),V14(:,2),V14(:,3),'.', 'MarkerEdgeColor',[255/255, 0/255, 0/255])
scatter3(non_V14(:,1),non_V14(:,2),non_V14(:,3),'.', 'MarkerEdgeColor',[180/255, 180/255, 180/255])
%scatter3(weighted_V14(:,1),weighted_V14(:,2),weighted_V14(:,3),'.', 'MarkerEdgeColor',[0/255, 0/255, 255/255])
hold off

weighted_tr

figure() % whole plotting - posture changed
hold on
axis equal
scatter3(COR(14:15,1),COR(14:15,2),COR(14:15,3),'*', 'MarkerEdgeColor',[0/255, 0/255, 0/255])
scatter3(non_V14(:,1),non_V14(:,2),non_V14(:,3),'.', 'MarkerEdgeColor',[180/255, 180/255, 180/255]) % non_V14
%scatter3(weighted_tr(:,1),weighted_tr(:,2),weighted_tr(:,3),'.', 'MarkerEdgeColor',[0/255, 0/255, 255/255])
scatter3(test(:,1),test(:,2),test(:,3),'.', 'MarkerEdgeColor',[0/255, 0/255, 255/255])
hold off

%% 

transformed = Body_temp;
transformed = skin_linear_body(transformed, transforms);
axes = bone_axes_body(transformed.COR);

%% Plotting joint 14 influence weights

weight14 = weights(:,14);
LI = weight14 ~= 0;
LII = weight14 == 0;
V14 = V(LI,:);
non_V14 = V(LII,:);

AT = zeros(2,3);
AT(1,:) = V14(1,:);
cc = (transforms{14}*v_mx)';
AT(2,:) = cc(1,1:3);

figure()
hold on
axis equal
scatter3(V14(2:end,1),V14(2:end,2),V14(2:end,3),'.', 'MarkerEdgeColor',[255/255, 0/255, 0/255])
scatter3(non_V14(:,1),non_V14(:,2),non_V14(:,3),'.', 'MarkerEdgeColor',[180/255, 180/255, 180/255])
scatter3(AT(:,1),AT(:,2),AT(:,3),'.', 'MarkerEdgeColor',[0/255, 0/255, 255/255])
hold off



%% after transformation

C = transformed.COR;
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
