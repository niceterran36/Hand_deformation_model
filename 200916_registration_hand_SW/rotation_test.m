

v_S_D3_MCB
v_S_D3_PPLX
D3_MCB_axis

o = [0 0 0]
z = [0 0 1];
AXIS = [o; v_S_D3_MCB; v_S_D3_PPLX; D3_MCB_axis; z];

figure()
axis equal
hold on
scatter3(o(:,1),o(:,2),o(:,3),'*','MarkerEdgeColor',[0/255, 0/255, 0/255]);
scatter3(v_S_D3_MCB(:,1),v_S_D3_MCB(:,2),v_S_D3_MCB(:,3),'o','MarkerEdgeColor',[0/255, 0/255, 255/255]);
scatter3(v_S_D3_PPLX(:,1),v_S_D3_PPLX(:,2),v_S_D3_PPLX(:,3),'o', 'MarkerEdgeColor',[0/255, 255/255, 0/255]);
scatter3(D3_MCB_axis(:,1),D3_MCB_axis(:,2),D3_MCB_axis(:,3),'*', 'MarkerEdgeColor',[190/255, 240/255, 251/255]);
plot3(AXIS(1:2,1),AXIS(1:2,2),AXIS(1:2,3),'-g');
plot3(AXIS([1 3],1),AXIS([1 3],2),AXIS([1 3],3),'-g');
plot3(AXIS([1 4],1),AXIS([1 4],2),AXIS([1 4],3),'-b');
plot3(AXIS([1 5],1),AXIS([1 5],2),AXIS([1 5],3),'-k');
hold off

% Z-axis rotation
vectorK = D3_MCB_axis(1,1:2);
vectorAxis = [0, 1]
rotationAngle_Z = acosd(dot(vectorK, vectorAxis) / (norm(vectorK) * norm(vectorAxis)));
Rz = function_rotationmat3D((-rotationAngle_Z)/180*pi, [0, 0, 1]);
D3_MCB_axis = function_rotation_matrix(D3_MCB_axis, Rz);
v_S_D3_MCB = function_rotation_matrix(v_S_D3_MCB, Rz);
v_S_D3_PPLX = function_rotation_matrix(v_S_D3_PPLX, Rz);
                                    
% X-axis rotation
vectorK = D3_MCB_axis(1,2:3);
vectorAxis = [0, 1]
rotationAngle_X = acosd(dot(vectorK, vectorAxis) / (norm(vectorK) * norm(vectorAxis)));
Rx = function_rotationmat3D((rotationAngle_X)/180*pi, [1, 0, 0]);
D3_MCB_axis = function_rotation_matrix(D3_MCB_axis, Rx);
v_S_D3_MCB = function_rotation_matrix(v_S_D3_MCB, Rx);
v_S_D3_PPLX = function_rotation_matrix(v_S_D3_PPLX, Rx);

s_angle_bt_MCB_PPLX = acosd(dot(v_S_D3_MCB, v_S_D3_PPLX) / (norm(v_S_D3_MCB) * norm(v_S_D3_PPLX)));


