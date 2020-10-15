
mesh_vertices = vertices;
points_vertices = points.vertices;
points_normals = points.normals;

transform = eye(4)
n = size(pairs, 1)

% Define derivative rotation matrix (linear approximation for small angles)
% ��̺� ������
Rx = [0, 0, 0; 0, 0, -1; 0, 1, 0] % x ��̺�
Ry = [0, 0, 1; 0, 0, 0; -1, 0, 0] % y ��̺�
Rz = [0, -1, 0; 1, 0, 0; 0, 0, 0] % z ��̺�

J = zeros(n, 7) % 1:3 for x, y, z �̺�, 4:6 for translation, 7 for scale factor
b = zeros(n, 1) % 1:3 for x, y, z �̺�, 4:6 for translation, 7 for scale factor

% question: J�� b�� �� �����ϴ��� �𸣰���
% J: normal�� mesh_vertices�� ���� �ľ�
% b: normal�� mesh_vertices - scan_points�� ���� �ľ�

% question: mesh_vertices�� normal�� scan_points�� normal�� mapping �ϴ°� ���� ������?


for i = 1 : n
    p = mesh_vertices(pairs(i, 1), :)
    q = points_vertices(pairs(i, 2), :)
    n = points_normals(pairs(i, 2), :)

    b(i) = n * (p - q)'; % b(i)�� cos(theta)�� �ǵ��� ���̶�� p-q vector�� normalization �ʿ�

    J(i, 1) = n * (Rx * p');
    J(i, 2) = n * (Ry * p');
    J(i, 3) = n * (Rz * p');
    J(i, 4 : 6) = n;
    J(i, 7) = 2 * n * p'; % scale factor �� �̷��� ���ϴ��� �𸣰���

end

% Create and solve linear system - ����� �����Բ� ����
JtJ = J' * J;
Jtb = J' * b;
result = JtJ \ -Jtb; % \������ JtJ�� ������� �ǹ� JtJ \ = inv(JtJ)

% Extract transform matrix
scale = (result(7) + 1) ^ 2; % scale factor �� �̷��� ���ϴ��� �𸣰���
rotation = ...
    rotation_matrix(result(1), [1, 0, 0]) * ...
    rotation_matrix(result(2), [0, 1, 0]) * ...
    rotation_matrix(result(3), [0, 0, 1]);
translation = result(4 : 6)';
transform = [diag(scale) * eye(3), translation'; 0, 0, 0, 1] * rotation;