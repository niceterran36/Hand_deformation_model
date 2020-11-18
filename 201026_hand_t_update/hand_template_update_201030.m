% vertices segmentation for dorsal, palmar

addpath(genpath('external'));
addpath('F:\[GitHub]\Hand_deformation_model\functions');
addpath('D:\GitHub\Hand_deformation_model\functions');
addpath('LM_for_dorsal_palmar')
load('hy_mesh_n5.mat'); %template
LMs = function_get_LM_from_iges('thumb_sep_LM.igs');

% assignment information
% D1 thumb. metacarpal: assignment 6
% D1 pro. phalanx: assignment 7
% D1 dis. phalanx: assignment 8

% D2 dis. phalanx: assignment 11
% D2 mid. phalanx: assignment 10
% D2 pro. phalanx: asisgnment 9
% D3 dis. phalanx: assignment 14
% D3 mid. phalanx: assignment 13
% D3 pro. phalanx: asisgnment 12

% D4 dis. phalanx: assignment 17
% D4 mid. phalanx: assignment 16
% D4 pro. phalanx: asisgnment 15
% D5 dis. phalanx: assignment 20
% D5 mid. phalanx: assignment 19
% D5 pro. phalanx: asisgnment 18

figure()
axis equal
axis off
hold on
view(-7,7);
scatter3(vertices(:,1),vertices(:,2),vertices(:,3),'.', 'MarkerEdgeColor',[180/255, 180/255, 180/255]);
scatter3(centers_c(:,1),centers_c(:,2),centers_c(:,3),'*','MarkerEdgeColor',[255/255, 0/255, 0/255]);
scatter3(LMs(:,1),LMs(:,2),LMs(:,3),'*','MarkerEdgeColor',[242/255, 150/255, 97/255]);
scatter3(Sg12(:,1),Sg12(:,2),Sg12(:,3),'.','MarkerEdgeColor',[243/255, 97/255, 166/255]);
hold off

% input points
p1 = LMs(1,:);
p2 = LMs(5,:);
p3 = LMs(6,:);
[a, b, c, d] = generate_plane_3point(p1, p2, p3);
PL_com = zeros(1,4);
PL_com = [a, b, c, d];
clear a b c d
% assignment_new = mesh.assignment;

LIX = mesh.assignment == 8;
V_IDX = [1:size(mesh.vertices,1)]';
Sg8 = vertices(LIX,:);
Sg8_idx = V_IDX(LIX,:);

% testing value
%stone = [8.6169, -7.8574, 95.0876];% D2 distal
%stone = [2.501, -5.676, 75.95];% D2 middle
%stone = [-6.016, -4.884, 47.99];% D2 proximal
%stone = [-22.39, -6.443, 95.31];% D3 distal
%stone = [-26.93, -4.14, 75.21];% D3 middle
%stone = [-20.03, 3.01, 53];% D3 proximal
stone = [39.20, -15.69, 26.81];% D1 distal

T = PL_com(1)*stone(1)+PL_com(2)*stone(2)+PL_com(3)*stone(3)+PL_com(4);

A = Sg8;
Compare = zeros(size(A,1),1);
for i = 1:size(A,1)
    T = PL_com(1)*A(i,1)+PL_com(2)*A(i,2)+PL_com(3)*A(i,3)+PL_com(4);
    Compare(i) = T;
end 
for k = 1:size(Compare,1)
    if Compare(k) > 0
       TT(k,1) = 108; % dorsal side
    elseif Compare(k) < 0
       TT(k,1) = 208; % palmar side
    else
       TT(k,1) = 208;
    end 
end 
assignment_new(Sg8_idx) = TT;

LIX_u = assignment_new == 108;
V_SgX_dor = vertices(LIX_u,:);

figure()
axis equal
axis off
hold on
view(-7,7);
scatter3(vertices(:,1),vertices(:,2),vertices(:,3),'.', 'MarkerEdgeColor',[180/255, 180/255, 180/255]);
%scatter3(centers_c(:,1),centers_c(:,2),centers_c(:,3),'*','MarkerEdgeColor',[255/255, 0/255, 0/255]);
%scatter3(LMs(:,1),LMs(:,2),LMs(:,3),'*','MarkerEdgeColor',[242/255, 150/255, 97/255]);
scatter3(V_SgX_dor(:,1),V_SgX_dor(:,2),V_SgX_dor(:,3),'.','MarkerEdgeColor',[243/255, 97/255, 166/255]);
hold off

%% for command update

dirLM = dir('D:\GitHub\Hand_deformation_model\201026_hand_t_update\LM_for_dorsal_palmar\*.igs');

Seg_order = [8 7 11 10 9 14 13 12 17 16 15 20 19 18];
Assignment_order = [208 207 111 110 109 114 113 112 117 116 115 120 119 118;
                    108 107 211 210 209 214 213 212 217 216 215 220 219 218]';

for ii = 1:2
    LMs = function_get_LM_from_iges(dirLM(ii).name);
    fprintf('%d, %s\n', i, dirLM(ii).name);
    
    p1 = LMs(1,:);
    p2 = LMs(2,:);
    p3 = LMs(3,:);
    [a, b, c, d] = generate_plane_3point(p1, p2, p3);
    PL_com = zeros(1,4);
    PL_com = [a, b, c, d];
    clear a b c d SgXf SgXf_idx
    
    LIX = mesh.assignment == Seg_order(ii);
    V_IDX = [1:size(mesh.vertices,1)]';
    SgXf = vertices(LIX,:);
    SgXf_idx = V_IDX(LIX,:);
    
    stone = [-26.93, -4.14, 75.21];% D3 middle
    T = PL_com(1)*stone(1)+PL_com(2)*stone(2)+PL_com(3)*stone(3)+PL_com(4);
    
    if T < 0
       dorsal_factor = -1;
    else 
       dorsal_factor = 1;
    end 
    
    A = SgXf;
    Compare = zeros(size(A,1),1);
    for i = 1:size(A,1)
        T = PL_com(1)*A(i,1)+PL_com(2)*A(i,2)+PL_com(3)*A(i,3)+PL_com(4);
        Compare(i) = T;
    end 
    Compare = dorsal_factor*Compare;
    
    for k = 1:size(Compare,1)
        if Compare(k) > 0
           TT(k,1) = Assignment_order(ii,1); % dorsal side
        elseif Compare(k) < 0
           TT(k,1) = Assignment_order(ii,2); % palmar side
        else
           TT(k,1) = Assignment_order(ii,1);
        end 
    end 
    assignment_new(SgXf_idx) = TT;
    
    LIX_u = assignment_new == Assignment_order(ii,1);
    V_SgX_dor = vertices(LIX_u,:);

    figure()
    axis equal
    axis off
    hold on
    view(-7,7);
    scatter3(vertices(:,1),vertices(:,2),vertices(:,3),'.', 'MarkerEdgeColor',[180/255, 180/255, 180/255]);
    scatter3(centers_c(:,1),centers_c(:,2),centers_c(:,3),'*','MarkerEdgeColor',[255/255, 0/255, 0/255]);
    %scatter3(LMs(:,1),LMs(:,2),LMs(:,3),'*','MarkerEdgeColor',[242/255, 150/255, 97/255]);
    scatter3(V_SgX_dor(:,1),V_SgX_dor(:,2),V_SgX_dor(:,3),'.','MarkerEdgeColor',[243/255, 97/255, 166/255]);
    hold off
    
    clear TT A Compare T LIX 
end 




