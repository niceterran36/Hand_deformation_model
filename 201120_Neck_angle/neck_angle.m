clc
clear all
addpath(genpath('../external'));
addpath('F:\[GitHub]\Hand_deformation_model\functions');
addpath('F:\[GitHub]\Hand_deformation_model\data_SW');
addpath('F:\[GitHub]\Hand_deformation_model\data');
addpath('F:\[GitHub]\Hand_deformation_model\external\registration');
format shortG

addpath('S02_박가온');
addpath('S03_박수아');
addpath('S03_박수아2');
addpath('S04_정도현');
addpath('S05_문아인');
addpath('S06_이서현');
addpath('S07_이현주');
addpath('S08_김한결');
addpath('S09_박지한');
addpath('S10_김아윤');
addpath('LM_data');
dirLM = dir('F:\[3D Head] 양산부산대 소아사경\LM_data\*.igs');

n = 21;
Angle = zeros(n,6);

% LM2 = [-137.39,85.41,-21.98; 
%     -122.38,49.10,18.56; 
%     -85.06,-11.54,31.18;
%     -168.43,-22.93,-81.06;
%     -27.52,54.28,-72.10;
%     -59.16,-144.09,-42.01;
%     -221.84,-155.03,-83.93;
%     116.42,-136.84,-90.63];

for i = 22:42
LM = function_get_LM_from_iges(dirLM(i).name);
LM(7,1:3) = (LM(2,:)+LM(3,:))/2;
LM(8,1:3) = (LM(5,:)+LM(6,:))/2;

% figure()
% hold on
% axis equal
% axis off
% plot3(LM(1, 1), LM(1, 2), LM(1, 3), 'r*');
% plot3(LM(2, 1), LM(2, 2), LM(2, 3), 'g*');
% plot3(LM(3, 1), LM(3, 2), LM(3, 3), 'r*');
% plot3(LM(4, 1), LM(4, 2), LM(4, 3), 'g*');
% plot3(LM(5, 1), LM(5, 2), LM(5, 3), 'g*');
% plot3(LM(6, 1), LM(6, 2), LM(6, 3), 'k*');
% plot3(LM(7, 1), LM(7, 2), LM(7, 3), 'b*');
% plot3(LM(8, 1), LM(8, 2), LM(8, 3), 'b*');
% plot3(LM([2 4], 1), LM([2 4], 2), LM([2 4], 3), 'k-');
% plot3(LM([2 5], 1), LM([2 5], 2), LM([2 5], 3), 'k-');
% plot3(LM(6:7, 1), LM(6:7, 2), LM(6:7, 3), 'k-');
% plot3(LM([6 8], 1), LM([6 8], 2), LM([6 8], 3), 'k-');
% hold off

figure()
hold on
axis equal
axis off
plot3(LM(1, 1), LM(1, 2), LM(1, 3), 'g*');
plot3(LM(2, 1), LM(2, 2), LM(2, 3), 'g*');
plot3(LM(3, 1), LM(3, 2), LM(3, 3), 'g*');
plot3(LM(4, 1), LM(4, 2), LM(4, 3), 'b*');
plot3(LM(5, 1), LM(5, 2), LM(5, 3), 'b*');
plot3(LM(6, 1), LM(6, 2), LM(6, 3), 'b*');
plot3(LM(7, 1), LM(7, 2), LM(7, 3), 'g*');
plot3(LM(8, 1), LM(8, 2), LM(8, 3), 'b*');
plot3(LM([1 2], 1), LM([1 2], 2), LM([1 2], 3), 'k-');
plot3(LM([1 3], 1), LM([1 3], 2), LM([1 3], 3), 'k-');
plot3(LM(4:5, 1), LM(4:5, 2), LM(4:5, 3), 'k-');
plot3(LM([4 6], 1), LM([4 6], 2), LM([4 6], 3), 'k-');
plot3(LM([1 7], 1), LM([1 7], 2), LM([1 7], 3), 'g-');
plot3(LM([4 8], 1), LM([4 8], 2), LM([4 8], 3), 'b-');
hold off

% v_f1 = (LM(4,:)-LM(2,:))/norm(LM(4,:)-LM(2,:));
% v_f2 = (LM(5,:)-LM(2,:))/norm(LM(5,:)-LM(2,:));
% v_t1 = (LM(7,:)-LM(6,:))/norm(LM(7,:)-LM(6,:));
% v_t2 = (LM(8,:)-LM(6,:))/norm(LM(8,:)-LM(6,:));

v_f1 = (LM(2,:)-LM(1,:))/norm(LM(2,:)-LM(1,:));
v_f2 = (LM(3,:)-LM(1,:))/norm(LM(3,:)-LM(1,:));
v_t1 = (LM(5,:)-LM(4,:))/norm(LM(5,:)-LM(4,:));
v_t2 = (LM(6,:)-LM(4,:))/norm(LM(6,:)-LM(4,:));
m_vf = (LM(7,:)-LM(1,:))/norm(LM(7,:)-LM(1,:));
m_vt = (LM(8,:)-LM(4,:))/norm(LM(8,:)-LM(4,:));

v_f = cross(v_f1,v_f2);
v_f = -v_f/norm(v_f);
v_t = cross(v_t1,v_t2);
v_t = -v_t/norm(v_t);

o = [0,0,0];
x = [1 0 0];
y = [0 1 0];
z = [0 0 1];

AXIS = [v_f; v_t; o; x; y; z; m_vf; m_vt];

% torsor orthogornal vector = blue, face orthogornal vector = green
figure()
hold on
axis equal
axis on
plot3(AXIS(1,1), AXIS(1,2), AXIS(1,3), 'g*');
plot3(AXIS(2,1), AXIS(2,2), AXIS(2,3), 'b*');
plot3(AXIS(3,1), AXIS(3,2), AXIS(3,3), 'k*');
%plot3(AXIS(7,1), AXIS(7,2), AXIS(7,3), 'c*');
plot3(AXIS(8,1), AXIS(8,2), AXIS(8,3), 'm*');
plot3(AXIS([1 3],1), AXIS([1 3],2), AXIS([1 3],3), 'g-');
plot3(AXIS([2 3],1), AXIS([2 3],2), AXIS([2 3],3), 'b-');
plot3(AXIS([3 4],1), AXIS([3 4],2), AXIS([3 4],3), 'k-');
plot3(AXIS([3 5],1), AXIS([3 5],2), AXIS([3 5],3), 'k-');
plot3(AXIS([3 6],1), AXIS([3 6],2), AXIS([3 6],3), 'k-');
plot3(AXIS([3 8],1), AXIS([3 8],2), AXIS([3 8],3), 'm-');
hold off

% %x-z angle
% v_f = AXIS(1,[1 3]);
% z_axis = [0 1];
% xz_angle = acosd(dot(v_f, z_axis) / (norm(v_f) * norm(z_axis)))

va = AXIS(8,[1 3]);
vt = [0 1];
Angle_y = acosd(dot(va, vt) / (norm(va) * norm(vt)))
Ry = function_rotationmat3D((-Angle_y)/180*pi, [0, 1, 0]);
AXIS([1 2 7 8],:) = function_rotation_matrix(AXIS([1 2 7 8],:), Ry);

%x-z angle
v_mf = AXIS(7,[1 3]);
v_mt = AXIS(8,[1 3]);
xz_angle = acosd(dot(v_mf , v_mt) / (norm(v_mf ) * norm(v_mt)))

%y-z angle
v_f = AXIS(1,[2 3]);
y_axis = [1 0];
yz_angle = acosd(dot(v_f, y_axis) / (norm(v_f) * norm(y_axis)))

%x-y angle
v_f = AXIS(1,[1 2]);
x_axis = [0 1];
xy_angle = acosd(dot(v_f, x_axis) / (norm(v_f) * norm(x_axis)))


va = AXIS(2,1:2);
vt = [0 1];
Angle_z = acosd(dot(va, vt) / (norm(va) * norm(vt)))
Rz = function_rotationmat3D((Angle_z)/180*pi, [0, 0, 1]);
AXIS([1 2 7 8],:) = function_rotation_matrix(AXIS([1 2 7 8],:), Rz);

va = AXIS(2,2:3);
vt = [1 0];
Angle_x = acosd(dot(va, vt) / (norm(va) * norm(vt)))
Rx = function_rotationmat3D((Angle_x)/180*pi, [1, 0, 0]);
AXIS([1 2 7 8],:) = function_rotation_matrix(AXIS([1 2 7 8],:), Rx);

%y-z angle
v_f_tr = AXIS(1,[2 3]);
x_axis = [1 0];
yz_angle_tr = acosd(dot(v_f_tr, x_axis) / (norm(v_f_tr) * norm(x_axis)))

%x-y angle
v_f_tr = AXIS(1,[1 2]);
y_axis = [0 1];
xy_angle_tr = acosd(dot(v_f_tr, y_axis) / (norm(v_f_tr) * norm(y_axis)))

%3D angle
v_f = AXIS(1,:);
v_t = AXIS(2,:);
Angle3D = acosd(dot(v_f, v_t) / (norm(v_f) * norm(v_t)))

Angle(i,:) = [xz_angle yz_angle xy_angle yz_angle_tr xy_angle_tr Angle3D];

% figure()
% hold on
% axis equal
% axis on
% plot3(AXIS(1,1), AXIS(1,2), AXIS(1,3), 'g*');
% plot3(AXIS(2,1), AXIS(2,2), AXIS(2,3), 'b*');
% plot3(AXIS(3,1), AXIS(3,2), AXIS(3,3), 'k*');
% %plot3(AXIS(7,1), AXIS(7,2), AXIS(7,3), 'c*');
% plot3(AXIS(8,1), AXIS(8,2), AXIS(8,3), 'm*');
% plot3(AXIS([1 3],1), AXIS([1 3],2), AXIS([1 3],3), 'g-');
% plot3(AXIS([2 3],1), AXIS([2 3],2), AXIS([2 3],3), 'b-');
% plot3(AXIS([3 4],1), AXIS([3 4],2), AXIS([3 4],3), 'k-');
% plot3(AXIS([3 5],1), AXIS([3 5],2), AXIS([3 5],3), 'k-');
% plot3(AXIS([3 6],1), AXIS([3 6],2), AXIS([3 6],3), 'k-');
% plot3(AXIS([3 8],1), AXIS([3 8],2), AXIS([3 8],3), 'm-');
% hold off

end 

