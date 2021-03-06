% register library - PC
addpath(genpath('../external'));
addpath('D:\GitHub\Hand_deformation_model\functions');
addpath('D:\GitHub\Hand_deformation_model\data_SW');
addpath('D:\GitHub\Hand_deformation_model\data');
addpath('D:\GitHub\Hand_deformation_model\external\registration');

load('hy_mesh_n3.mat');
vertices = mesh.vertices;
assignment = mesh.assignment;
centers = mesh.centers;

LMt = function_get_LM_from_iges('LM_for_palm_seg.igs');

%% 
figure()
axis equal
hold on
scatter3(vertices(:,1),vertices(:,2),vertices(:,3),'.','MarkerEdgeColor',[200/255, 200/255, 200/255]);
scatter3(LMt(1:3,1),LMt(1:3,2),LMt(1:3,3),'*','MarkerEdgeColor',[0/255, 0/255, 255/255]);
scatter3(stone(1,1),stone(1,2),stone(1,3),'*','MarkerEdgeColor',[255/255, 0/255, 0/255]);
% scatter3(AXIS_D4(3,1),AXIS_D4(3,2),AXIS_D4(3,3),'*', 'MarkerEdgeColor',[255/255, 0/255, 0/255]);
% plot3(AXIS_D4(1:2,1),AXIS_D4(1:2,2),AXIS_D4(1:2,3),'-g');
% plot3(AXIS_D4([1 3],1),AXIS_D4([1 3],2),AXIS_D4([1 3],3),'-r');
% plot3(AXIS_D4([1 4],1),AXIS_D4([1 4],2),AXIS_D4([1 4],3),'-b');
% plot3(AXIS_D4([1 5],1),AXIS_D4([1 5],2),AXIS_D4([1 5],3),'-k');
% plot3(AXIS_D4([1 6],1),AXIS_D4([1 6],2),AXIS_D4([1 6],3),'-k');
% plot3(AXIS_D4([1 7],1),AXIS_D4([1 7],2),AXIS_D4([1 7],3),'-k');
view(0,5);
hold off

%% D2 separation 
% separation of thumb
% 3D points of plane for thumb separation
p1 = LMt(1,:);
p2 = LMt(2,:);
p3 = LMt(3,:);
p4 = LMt(4,:);
p5 = LMt(5,:);
p6 = LMt(6,:);
p7 = LMt(7,:);
p8 = LMt(8,:);
p9 = LMt(9,:);

[a1, b1, c1, d1] = generate_plane_3point(p1, p2, p3);
[a2, b2, c2, d2] = generate_plane_3point(p3, p4, p5);
[a3, b3, c3, d3] = generate_plane_3point(p5, p6, p7);
[a4, b4, c4, d4] = generate_plane_3point(p7, p8, p9);

PL = zeros(4,4);
PL(1,:) = [a1, b1, c1, d1];
PL(2,:) = [a2, b2, c2, d2];
PL(3,:) = [a3, b3, c3, d3];
PL(4,:) = [a4, b4, c4, d4];

stone = [2.87 -14.265 27.15];
stone2 = [-18.88 7.89 27.44];
stone3 = [-34.13 -21.81 18.36];
stone4 = [-53.42 -4.21 6.73]; 
TV1 = PL(1,1)*stone(1) + PL(1,2)*stone(2) + PL(1,3)*stone(3) + PL(1,4);
TV2 = PL(2,1)*stone2(1) + PL(2,2)*stone2(2) + PL(2,3)*stone2(3) + PL(2,4);
TV3 = PL(3,1)*stone3(1) + PL(3,2)*stone2(2) + PL(3,3)*stone3(3) + PL(3,4);
TV4 = PL(4,1)*stone4(1) + PL(4,2)*stone2(2) + PL(4,3)*stone4(3) + PL(4,4);

V_idx = [1:size(vertices,1)]';

% D2 segmentation 
LIX = assignment == 9;
D2_seg(:,1) = V_idx(LIX,:);
D2_seg(:,2:4) = vertices(LIX,:);

A = D2_seg(:,2:4);
Compare = zeros(size(A,1),1);
for i = 1:size(A,1)
    T = PL(1,1)*A(i,1)+PL(1,2)*A(i,2)+PL(1,3)*A(i,3)+PL(1,4);
    Compare(i) = T;
end 
Compare(Compare<0) = 0; % convert negative value as zero
TT = find(Compare) % index of compare > 0 

for i = 1:size(TT,1)
    assignment(D2_seg(TT(i))) = 2;
end 
clear A Compare T TT LIX 

% D3 segmentation 
LIX = assignment == 12;
D3_seg(:,1) = V_idx(LIX,:);
D3_seg(:,2:4) = vertices(LIX,:);

A = D3_seg(:,2:4);
Compare = zeros(size(A,1),1);
for i = 1:size(A,1)
    T = PL(2,1)*A(i,1)+PL(2,2)*A(i,2)+PL(2,3)*A(i,3)+PL(2,4);
    Compare(i) = T;
end 
Compare(Compare<0) = 0; % convert negative value as zero
TT = find(Compare) % index of compare > 0 

for i = 1:size(TT,1)
    assignment(D3_seg(TT(i))) = 3;
end 
clear A Compare T TT LIX 

% D4 segmentation 
LIX = assignment == 15;
D4_seg(:,1) = V_idx(LIX,:);
D4_seg(:,2:4) = vertices(LIX,:);

A = D4_seg(:,2:4);
Compare = zeros(size(A,1),1);
for i = 1:size(A,1)
    T = PL(3,1)*A(i,1)+PL(3,2)*A(i,2)+PL(3,3)*A(i,3)+PL(3,4);
    Compare(i) = T;
end 
Compare(Compare<0) = 0; % convert negative value as zero
TT = find(Compare) % index of compare > 0 

for i = 1:size(TT,1)
    assignment(D4_seg(TT(i))) = 4;
end 
clear A Compare T TT LIX 

% D5 segmentation 
LIX = assignment == 18;
D5_seg(:,1) = V_idx(LIX,:);
D5_seg(:,2:4) = vertices(LIX,:);

A = D5_seg(:,2:4);
Compare = zeros(size(A,1),1);
for i = 1:size(A,1)
    T = PL(4,1)*A(i,1)+PL(4,2)*A(i,2)+PL(4,3)*A(i,3)+PL(4,4);
    Compare(i) = T;
end 
Compare(Compare<0) = 0; % convert negative value as zero
TT = find(Compare) % index of compare > 0 

for i = 1:size(TT,1)
    assignment(D5_seg(TT(i))) = 5;
end 
clear A Compare T TT LIX 

%% data check 
% segment 9 = D2 metacarpal bone 

V2 = vertices;
v_segment = assignment;
VLI = [];
for segment=1:22
temLI = v_segment == segment;
VLI = [VLI temLI];
end 

temLI = v_segment == 1; Sg1 = V2(temLI,:);
temLI = v_segment == 2; Sg2 = V2(temLI,:);
temLI = v_segment == 3; Sg3 = V2(temLI,:);
temLI = v_segment == 4; Sg4 = V2(temLI,:);
temLI = v_segment == 5; Sg5 = V2(temLI,:);
temLI = v_segment == 6; Sg6 = V2(temLI,:);
temLI = v_segment == 7; Sg7 = V2(temLI,:);
temLI = v_segment == 8; Sg8 = V2(temLI,:);
temLI = v_segment == 9; Sg9 = V2(temLI,:);
temLI = v_segment == 10; Sg10 = V2(temLI,:);
temLI = v_segment == 11; Sg11 = V2(temLI,:);
temLI = v_segment == 12; Sg12 = V2(temLI,:);
temLI = v_segment == 13; Sg13 = V2(temLI,:);
temLI = v_segment == 14; Sg14 = V2(temLI,:);
temLI = v_segment == 15; Sg15 = V2(temLI,:);
temLI = v_segment == 16; Sg16 = V2(temLI,:);
temLI = v_segment == 17; Sg17 = V2(temLI,:);
temLI = v_segment == 18; Sg18 = V2(temLI,:);
temLI = v_segment == 19; Sg19 = V2(temLI,:);
temLI = v_segment == 20; Sg20 = V2(temLI,:);
temLI = v_segment == 21; Sg21 = V2(temLI,:);
temLI = v_segment == 22; Sg22 = V2(temLI,:);

figure()
hold on
axis equal
% scatter3(Sg1(:,1),Sg1(:,2),Sg1(:,3),'.', 'MarkerEdgeColor',[16/255, 241/255, 255/255])
% scatter3(Sg2(:,1),Sg2(:,2),Sg2(:,3),'.', 'MarkerEdgeColor',[213/255, 42/255, 219/255])
% scatter3(Sg3(:,1),Sg3(:,2),Sg3(:,3),'.', 'MarkerEdgeColor',[233/255, 30/255, 68/255])
% scatter3(Sg4(:,1),Sg4(:,2),Sg4(:,3),'.', 'MarkerEdgeColor',[179/255, 59/255, 235/255])
% scatter3(Sg5(:,1),Sg5(:,2),Sg5(:,3),'.', 'MarkerEdgeColor',[69/255, 204/255, 104/255])
% scatter3(Sg6(:,1),Sg6(:,2),Sg6(:,3),'.', 'MarkerEdgeColor',[191/255, 247/255, 20/255])
% scatter3(Sg7(:,1),Sg7(:,2),Sg7(:,3),'.', 'MarkerEdgeColor',[247/255, 170/255, 20/255])
% scatter3(Sg8(:,1),Sg8(:,2),Sg8(:,3),'.', 'MarkerEdgeColor',[242/255, 62/255, 27/255])
scatter3(Sg9(:,1),Sg9(:,2),Sg9(:,3),'.', 'MarkerEdgeColor',[191/255, 247/255, 255/255])
% scatter3(Sg10(:,1),Sg10(:,2),Sg10(:,3),'.', 'MarkerEdgeColor',[247/255, 170/255, 20/255])
% scatter3(Sg11(:,1),Sg11(:,2),Sg11(:,3),'.', 'MarkerEdgeColor',[242/255, 62/255, 27/255])
scatter3(Sg12(:,1),Sg12(:,2),Sg12(:,3),'.', 'MarkerEdgeColor',[191/255, 247/255, 20/255])
% scatter3(Sg13(:,1),Sg13(:,2),Sg13(:,3),'.', 'MarkerEdgeColor',[247/255, 170/255, 20/255])
% scatter3(Sg14(:,1),Sg14(:,2),Sg14(:,3),'.', 'MarkerEdgeColor',[242/255, 62/255, 27/255])
scatter3(Sg15(:,1),Sg15(:,2),Sg15(:,3),'.', 'MarkerEdgeColor',[191/255, 247/255, 255/255])
% scatter3(Sg16(:,1),Sg16(:,2),Sg16(:,3),'.', 'MarkerEdgeColor',[247/255, 170/255, 20/255])
% scatter3(Sg17(:,1),Sg17(:,2),Sg17(:,3),'.', 'MarkerEdgeColor',[242/255, 62/255, 27/255])
scatter3(Sg18(:,1),Sg18(:,2),Sg18(:,3),'.', 'MarkerEdgeColor',[191/255, 247/255, 20/255])
% scatter3(Sg19(:,1),Sg19(:,2),Sg19(:,3),'.', 'MarkerEdgeColor',[247/255, 170/255, 20/255])
% scatter3(Sg20(:,1),Sg20(:,2),Sg20(:,3),'.', 'MarkerEdgeColor',[242/255, 62/255, 27/255])
% scatter3(Sg21(:,1),Sg21(:,2),Sg21(:,3),'.', 'MarkerEdgeColor',[0, 0, 0])
%scatter3(Sg22(:,1),Sg22(:,2),Sg22(:,3),'.', 'MarkerEdgeColor',[0, 0, 0])
view(0,5);
hold off

%% 
figure()
hold on
axis equal
scatter3(D5_seg(:,2),v_thumb(:,3),v_thumb(:,4),'.', 'MarkerEdgeColor',[180/255, 180/255, 180/255])
hold off
mesh.assignment = assignment
