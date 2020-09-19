addpath(genpath('../external'));
addpath('C:\Users\user\Documents\MATLAB\functions');
addpath('C:\Users\user\Documents\MATLAB\data_SW');
addpath('C:\Users\user\Documents\MATLAB\data');
addpath('C:\Users\user\Documents\MATLAB\external\registration');

o = [0 0 0];
x = [1 0 0];
y = [0 1 0];
z = [0 0 1];
a = [-3 2 1];
a = [-3 2 1]./norm(a);

AXIS = [o; x; y; z; a];


figure()
axis equal
hold on
scatter3(AXIS(1:4,1),AXIS(1:4,2),AXIS(1:4,3),'*','MarkerEdgeColor',[0/255, 0/255, 0/255]);
%scatter3(D3_MCB_axis(:,1),D3_MCB_axis(:,2),D3_MCB_axis(:,3),'*', 'MarkerEdgeColor',[190/255, 240/255, 251/255]);
plot3(AXIS(1:2,1),AXIS(1:2,2),AXIS(1:2,3),'-k');
plot3(AXIS([1 3],1),AXIS([1 3],2),AXIS([1 3],3),'-k');
plot3(AXIS([1 4],1),AXIS([1 4],2),AXIS([1 4],3),'-k');
plot3(AXIS([1 5],1),AXIS([1 5],2),AXIS([1 5],3),'-b');
plot3([0; a_tr(1)],[0; a_tr(2)],[0; a_tr(3)],'-b');
hold off

va = a(1:2)
vt = [0 1]
Angle_z = acosd(dot(va, vt) / (norm(va) * norm(vt)));
Rz = function_rotationmat3D((-Angle_z)/180*pi, [0, 0, 1]);
a_tr = a'
a_tr = Rz*a_tr
a_tr(1) = round(a_tr(1))

va = a_tr(2:3)
vt = [0 1]
Angle_x = acosd(dot(va, vt) / (norm(va) * norm(vt)));
Rx = function_rotationmat3D((Angle_x)/180*pi, [1, 0, 0]);
a_tr = Rx*a_tr

figure()
axis equal
hold on
scatter3(AXIS(1:4,1),AXIS(1:4,2),AXIS(1:4,3),'*','MarkerEdgeColor',[0/255, 0/255, 0/255]);
%scatter3(D3_MCB_axis(:,1),D3_MCB_axis(:,2),D3_MCB_axis(:,3),'*', 'MarkerEdgeColor',[190/255, 240/255, 251/255]);
plot3(AXIS(1:2,1),AXIS(1:2,2),AXIS(1:2,3),'-k');
plot3(AXIS([1 3],1),AXIS([1 3],2),AXIS([1 3],3),'-k');
plot3(AXIS([1 4],1),AXIS([1 4],2),AXIS([1 4],3),'-k');
plot3(AXIS([1 5],1),AXIS([1 5],2),AXIS([1 5],3),'-b');
plot3([0; a_tr(1)],[0; a_tr(2)],[0; a_tr(3)],'-b');
hold off




















